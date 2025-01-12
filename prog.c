#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/select.h>
#include <time.h>

// Defines
#define SERVER_IP "192.168.159.196" // Cambia esta IP por la del ESP32
#define SERVER_PORT 3333           // Cambia este puerto por el configurado en tu servidor
#define TIMEOUT_SECONDS 10         // Tiempo máximo de espera para la respuesta
#define BUF_SIZE 11

// Macros
#ifdef _WIN32
    #define CLEAR_SCREEN() system("cls")
#else
    #define CLEAR_SCREEN() system("clear")
#endif

// Prototipos
void *wifi_communication(void *arg);
void *serial_communication(void *arg);
void configure_serial_port(int *fd, const char *port);
void configurar_reles();
void inicializacion_socket();
void enviar_comando(const char *comando);
void log_comando(const char *comando);

// Variables
int sockfd;
struct sockaddr_in server_addr;
char message[BUF_SIZE];
char buffer[BUF_SIZE];
socklen_t addr_len = sizeof(server_addr);
int estado_reles[4] = {0, 0, 0, 0}; // 0 = apagado, 1 = encendido

// Menú principal
int main() {
    pthread_t wifi_thread, serial_thread;
    int choice;

    while (1) {
        CLEAR_SCREEN(); // Limpiar pantalla al entrar al menú

        printf("\n--- Menu de Comunicaciones---\n");
        printf("1. Comunicacion por Wi-Fi\n");
        printf("2. Comunicacion por Serial\n");
        printf("0. Salir\n");
        printf("Seleccione una opcion: ");
        scanf("%d", &choice);

        switch (choice) {
            case 1:
                if (pthread_create(&wifi_thread, NULL, wifi_communication, NULL) != 0) {
                    perror("Failed to create Wi-Fi thread");
                } else {
                    pthread_join(wifi_thread, NULL); // Espera a que el hilo termine
                }
                break;
            case 2:
                if (pthread_create(&serial_thread, NULL, serial_communication, NULL) != 0) {
                    perror("Failed to create Serial thread");
                } else {
                    pthread_join(serial_thread, NULL); // Espera a que el hilo termine
                }
                break;
            case 0:
                printf("Exiting program...\n");// Buffer para el comando
                exit(0);
            default:
                printf("Invalid option. Please try again.\n");
        }

        sleep(3); // Esperar 3 segundos antes de volver al menú principal
    }

    return 0;
}

// Inicialización del socket UDP
void inicializacion_socket() {
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Error al crear el socket");
        exit(EXIT_FAILURE);
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
        perror("Dirección inválida o no soportada");
        exit(EXIT_FAILURE);
    }
}

// Enviar comando al ESP32
void enviar_comando(const char *comando) {
    if (sendto(sockfd, comando, strlen(comando), 0, (struct sockaddr *)&server_addr, addr_len) < 0) {
        perror("Error al enviar el mensaje");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    printf("Comando enviado: %s\n", comando);
    log_comando(comando);
}

// Registrar comando en archivo log
void log_comando(const char *comando) {
    FILE *log_file = fopen("comandos_log.txt", "a"); // Abrir en modo append
    if (!log_file) {
        perror("Error al abrir el archivo de log");
        return;
    }

    time_t now = time(NULL);
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", localtime(&now));

    fprintf(log_file, "%s - Comando: %s\n", timestamp, comando);
    fclose(log_file);
}

// Configurar relés
void configurar_reles() {
    printf("\n--- Configuración de Relés ---\n");
    for (int i = 0; i < 4; i++) {
        printf("Relé %d (1=ON, 0=OFF): ", i + 1);
        scanf("%d", &estado_reles[i]);
        if (estado_reles[i] != 0 && estado_reles[i] != 1) {
            printf("Entrada inválida. Relé %d apagado por defecto.\n", i + 1);
            estado_reles[i] = 0;
        }
    }
    printf("\nEstado de los relés actualizado:\n");
    for (int i = 0; i < 4; i++) {
        printf("Relé %d: %s\n", i + 1, estado_reles[i] ? "Encendido" : "Apagado");
    }
}

// Función para comunicación Wi-Fi (UDP)
void *wifi_communication(void *arg) {
    char comando[BUF_SIZE]; 
    inicializacion_socket();

    // Configuración de relés
    configurar_reles();

    // Actualizar el string del comando con los valores de los relés
    snprintf(comando, BUF_SIZE, "WI:%d,%d,%d,%d", 
             estado_reles[0], 
             estado_reles[1], 
             estado_reles[2], 
             estado_reles[3]);

    // Enviar el comando al ESP32
    enviar_comando(comando);

    close(sockfd); // Cerrar el socket
    pthread_exit(NULL);
}

// Función para comunicación Serial
void *serial_communication(void *arg) {
   int serial_port = open("/dev/ttyACM0", O_RDWR); // Change to the appropriate serial port
   if (serial_port < 0) {
       perror("Failed to open the serial port");
       return 1;
   }
   struct termios tty;
   if (tcgetattr(serial_port, &tty) != 0) {
       perror("Failed to get attributes");
       close(serial_port);
       return 1;
   }
   // Configure the serial port
   cfsetispeed(&tty, B115200);
   cfsetospeed(&tty, B115200);
   tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
   tty.c_iflag &= ~IGNBRK; // Disable break processing
   tty.c_lflag = 0; // No signaling chars, no echo, no canonical processing
   tty.c_oflag = 0; // No remapping, no delays
   tty.c_cc[VMIN] = 1; // Read blocks
   tty.c_cc[VTIME] = 1; // Read timeout (tenths of a second)
   tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Shut off XON/XOFF ctrl
   tty.c_cflag |= (CLOCAL | CREAD); // Ignore modem controls, enable reading
   tty.c_cflag &= ~(PARENB | PARODD); // Disable parity
   tty.c_cflag &= ~CSTOPB; // 1 stop bit
   tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS flow control
   if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
       perror("Failed to set attributes");
       close(serial_port);
       return 1;
   }
   // read from the serial port
   while(1)
   {
       char buffer[5];
       memset(buffer, 0, sizeof(buffer));
       int num_bytes = read(serial_port, buffer, sizeof(buffer));
       if (num_bytes == 5) {
           printf("Read %d bytes: %s\n", num_bytes, buffer);
       }
   }
   close(serial_port);
   return 0;
}


