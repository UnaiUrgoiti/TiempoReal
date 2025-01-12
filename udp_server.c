    /*########################################################################################*/
    /*Cabeceras*/
    /*########################################################################################*/
    #include <string.h>
    #include <sys/param.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_system.h"
    #include "esp_wifi.h"
    #include "esp_event.h"
    #include "esp_log.h"
    #include "nvs_flash.h"
    #include "esp_netif.h"
    #include "protocol_examples_common.h"

    #include "lwip/err.h"
    #include "lwip/sockets.h"
    #include "lwip/sys.h"
    #include <lwip/netdb.h>

    #include "driver/i2c.h"
    #include "driver/uart.h"
    #include "driver/gpio.h"
    #include "driver/spi_master.h"

    /*########################################################################################*/
    /*FDeclaracion de variables y definiciones*/
    /*########################################################################################*/


    #define I2C_MASTER_SCL_IO    22 // Pin SCL
    #define I2C_MASTER_SDA_IO    21 // Pin SDA
    #define I2C_MASTER_NUM       I2C_NUM_0
    #define I2C_MASTER_FREQ_HZ   100000
 //   #define I2C_SLAVE_ADDR       0x4D // Dirección del esclavo
    #define PORT CONFIG_EXAMPLE_PORT
    #define LED_GPIO GPIO_NUM_25       // Pin del LED interno
    #define TAM_BUFF 11

    static const char *TAG = "TiempoReal";
    uint8_t mensaje_enviar[TAM_BUFF] = "WI:1,1,1,1";

    uint8_t I2C_SLAVE_ADDR;

    esp_err_t i2c_master_send(const uint8_t *data, size_t data_len);
    static void led_blink(void) ;
    static void i2c_master_init(void);
    static void i2c_task(void *pvParameters);
    static void send_initial_message(void);

/*########################################################################################*/
/*Configuracion y envio datos por UART*/
/*########################################################################################*/

#define UART_PORT UART_NUM_1
#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define UART_BAUD_RATE 115200
#define UART_BUFFER_SIZE (1024)

static void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUFFER_SIZE, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "UART inicializado en TX:%d RX:%d a %d baud", UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
}

static void uart_send_data(const uint8_t *data) {
    int len = strlen((const char *)data);
    int sent = uart_write_bytes(UART_PORT, data, len);
    if (sent == len) {
        ESP_LOGI(TAG, "Datos enviados por UART: %s", data);
    } else {
        ESP_LOGE(TAG, "Error al enviar datos por UART");
    }
}


static void uart_periodic_task(void *pvParameters) {

    while (1) {
        uart_send_data(mensaje_enviar); // Función que ya definimos para enviar datos por UART
        ESP_LOGI(TAG, "Mensaje enviado: %s", mensaje_enviar);
        vTaskDelay(pdMS_TO_TICKS(2000)); // Esperar 2 segundos
    }
}

    /*########################################################################################*/
    /*Configuracion SPI*/
    /*########################################################################################*/

#define SPI_CLK_SPEED 1000000
#define SPI_PIN_MISO 19
#define SPI_PIN_MOSI 23
#define SPI_PIN_CLK  18
#define SPI_PIN_CS   5
#define STRING_SIZE 11  // Tamaño fijo del string (11 bytes)
uint8_t tx_data[STRING_SIZE] ;

static spi_device_handle_t spi;

static esp_err_t spi_master_init(void) {
    esp_err_t ret;
    spi_bus_config_t bus_config = {
        .miso_io_num = SPI_PIN_MISO,
        .mosi_io_num = SPI_PIN_MOSI,
        .sclk_io_num = SPI_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    
    ret = spi_bus_initialize(HSPI_HOST, &bus_config, SPI_DMA_DISABLED);  // Desactivar DMA
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "SPI bus initialization failed");
        return ret;
    }

    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = SPI_CLK_SPEED,
        .mode = 1 ,
        .spics_io_num = SPI_PIN_CS,
        .queue_size = 7
    };

    ret = spi_bus_add_device(HSPI_HOST, &dev_config, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "Adding SPI device failed");
        return ret;
    }
    return ESP_OK;
}

static void spi_task(void *pvParameters) {

    spi_transaction_t t = {
        .length = STRING_SIZE * 8,  // Enviar 11 bytes (8 bits cada uno)
        .tx_buffer = mensaje_enviar,       // Buffer de datos a enviar
        .rx_buffer = NULL           // No necesitamos recibir nada
    };

    while (1) {
        esp_err_t ret = spi_device_transmit(spi, &t);  // Enviar datos
        if (ret != ESP_OK) {
            ESP_LOGE("SPI", "Error en la transmisión SPI");
        } else {
            ESP_LOGI("SPI", "Datos enviados: %s", mensaje_enviar);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));  
    }
}


    /*########################################################################################*/
    /*Busqueda dispositivos I2C*/
    /*########################################################################################*/

static esp_err_t i2c_master_probe(uint8_t address) {
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

static void search_for_i2c_device(void) {
    ESP_LOGI(TAG, "Buscando dispositivos I2C...");
    for (uint8_t address = 0x08; address <= 0x77; address++) {
        if (i2c_master_probe(address) == ESP_OK) {
            ESP_LOGI(TAG, "Dispositivo encontrado en la dirección I2C: 0x%02X", address);
            I2C_SLAVE_ADDR = address;  
            send_initial_message();
            return; 
        }
    }
    ESP_LOGE(TAG, "No se encontraron dispositivos I2C.");
}

    /*########################################################################################*/
    /*Funciones de la comunicacion I2C con el stm32*/
    /*########################################################################################*/

static void send_initial_message(void) {

    esp_err_t ret = i2c_master_send(mensaje_enviar, sizeof(mensaje_enviar));  // Solo enviamos los datos
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Mensaje de inicialización enviado por I2C");
    } else {
        ESP_LOGE(TAG, "Error al enviar mensaje de inicialización por I2C");
    }
    led_blink();
    led_blink();
}

    static void i2c_master_init(void) {
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

static void led_blink(void) {
    gpio_set_level(LED_GPIO, 1); // Enciende el LED
    vTaskDelay(pdMS_TO_TICKS(100)); // Mantiene el LED encendido por 100 ms
    gpio_set_level(LED_GPIO, 0); // Apaga el LED
}

esp_err_t i2c_master_send(const uint8_t *data, size_t data_len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Enviar datos al esclavo
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x4D << 1) | I2C_MASTER_WRITE, true); 
    i2c_master_write(cmd, data, data_len, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    led_blink();  // Función para parpadear el LED (opcional)

    return ret;
}

static void i2c_task(void *pvParameters) {
    while (1) {
        // Enviar los datos por I2C
        ESP_LOGI(TAG, "Enviando datos por I2C: %s", mensaje_enviar);
        esp_err_t ret = i2c_master_send(mensaje_enviar, sizeof(mensaje_enviar));

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Mensaje enviado correctamente.");
        } else {
            ESP_LOGE(TAG, "Error al enviar el mensaje: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));  // Espera antes de la próxima iteración
    }
}


/*########################################################################################*/
/*Funciones del servidor UDP con Wi-Fi*/
/*########################################################################################*/

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[TAM_BUFF];
    char addr_str[TAM_BUFF];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        int enable = 1;
        lwip_setsockopt(sock, IPPROTO_IP, IP_PKTINFO, &enable, sizeof(enable));
#endif

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }2048
#endif
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        struct iovec iov;
        struct msghdr msg;
        struct cmsghdr *cmsgtmp;
        u8_t cmsg_buf[CMSG_SPACE(sizeof(struct in_pktinfo))];

        iov.iov_base = rx_buffer;
        iov.iov_len = sizeof(rx_buffer);
        msg.msg_control = cmsg_buf;
        msg.msg_controllen = sizeof(cmsg_buf);
        msg.msg_flags = 0;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_name = (struct sockaddr *)&source_addr;
        msg.msg_namelen = socklen;
#endif

        while (1) {

            ESP_LOGI(TAG, "Waiting for data");

            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
                    for ( cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp) ) {
                        if ( cmsgtmp->cmsg_level == IPPROTO_IP && cmsgtmp->cmsg_type == IP_PKTINFO ) {
                            struct in_pktinfo *pktinfo;
                            pktinfo = (struct in_pktinfo*)CMSG_DATA(cmsgtmp);
                            ESP_LOGI(TAG, "dest ip: %s", inet_ntoa(pktinfo->ipi_addr));
                        }
                    }
#endif
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; 
                
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
                
                memcpy(mensaje_enviar,rx_buffer, len);

                led_blink();
                led_blink();

                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }

    vTaskDelete(NULL);
}


/*########################################################################################*/
/*Main del Programa*/
/*########################################################################################*/


void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    // Inicialización de Comunicaciones
    i2c_master_init();
    //spi_master_init();
    uart_init();

    //Envio de mensaje inicial
    send_initial_message();

    // Configuración del LED como salida
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << LED_GPIO, // Configura el GPIO 2
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Iniciar tarea de I2C
    xTaskCreate(i2c_task, "i2c_task", 2048, NULL, 10, NULL);

    //Iniciar tarea del servidor UDP
    xTaskCreate(udp_server_task, "udp_server_task", 8192, (void *)AF_INET, 5, NULL);

    //Iniciar tarea de SPI
    ESP_ERROR_CHECK(spi_master_init());
    xTaskCreate(spi_task, "spi_task", 4096, NULL, 5, NULL);

    // Crear la tarea para enviar datos  por UART
    xTaskCreate(uart_periodic_task, "uart_periodic_task", 2048, NULL, 5, NULL);
}