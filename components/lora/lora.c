#include "lora.h"
#include <assert.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*

    TRANSMISOR LORA 
    AGRONECK

*/

static const char *TAG = "LoRa";

spi_device_handle_t lora_spi;

static void lora_spi_init() {
    // configuracion básica
    spi_bus_config_t buscfg = {
        .mosi_io_num = LORA_MOSI,  // GPIO 23
        .miso_io_num = LORA_MISO,  // GPIO 19
        .sclk_io_num = LORA_CLK,   // GPIO 18
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5*1000*1000,
        .mode = 0,
        .spics_io_num = LORA_CS,
        .queue_size = 7,
    };
    
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &lora_spi));
}

// Función para reiniciar el módulo
static void lora_reset(void) {
    gpio_set_level(LORA_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static uint8_t lora_read_register(uint8_t reg) {
    uint8_t tx_data[2] = { reg & 0x7F, 0x00 };
    uint8_t rx_data[2] = {0};
    spi_transaction_t t = {
        .length = 2 * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };

    spi_device_polling_transmit(lora_spi, &t);
    return rx_data[1];
}

// Funcion de escritura registros fifo LoRA( MSB + adress // payload )
static void lora_write_register(uint8_t address, uint8_t payload){
    uint8_t tx_data[2] = { address | 0x80, payload }; 
    spi_transaction_t t = {
        .length = 2 * 8,
        .tx_buffer = tx_data,
        .rx_buffer = NULL
    };
    
    esp_err_t ret = spi_device_polling_transmit(lora_spi, &t);
    assert(ret == ESP_OK);
}

// Función para enviar una cadena
static void lora_send_packetb(const uint8_t *data, size_t length) {
    // defino largo del payload
    lora_write_register(0x22, length);

    // configuro direcciones del FIFO
    lora_write_register(0x0E, 0x00);
    lora_write_register(0x0D, 0x00);

    // burst write
    uint8_t *spi_buf = heap_caps_malloc(length+1,MALLOC_CAP_DMA);
    if (spi_buf == NULL) {
        ESP_LOGE(TAG,"Error: No se pudo asignar memoria buffer SPI");
        return;
    }

    spi_buf[0] = 0x00 | 0x80; 
    memcpy(&spi_buf[1],data,length); // copio estructura al buffer

    spi_transaction_t t = {
        .length = (length+1)*8, // largo en bits
        .tx_buffer = spi_buf,
        .rx_buffer = NULL
    };

    spi_device_polling_transmit(lora_spi, &t); // transito sin soltar CS
    free(spi_buf); // libero memoria

    lora_write_register(0x12,0xFF); // limpio flags irq antes de transmitir
    lora_write_register(0x01,0x83); // regop: lora+tx

    uint8_t irq_flags;
    int timeout = 1000;
    while (timeout--) {
        irq_flags = lora_read_register(0x12);
        if (irq_flags & 0x08) { // detecta envio
            ESP_LOGI(TAG,"Paquete binario enviado (%d bytes)",length);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    lora_write_register(0x12,0xFF); // limpio flags irq despues de transmitir

    lora_write_register(0x01, 0x80); // regop: sleep
}

void transmitir_datos(payload_t *paquete){
    //payload_t paquete; // creo el paquete
    
    /*
        // asigno datos, ej:
    uint16_t id_local = 122;
    uint8_t  pulsaciones   = 82;
    float    temp_sensor   = 30.85;
    double   gps_latitud   = -42.0690;
    double   gps_longitud  = -67.6767; // datos simulados
    */
    
    /*
        // mapeo y empaqueto
    paquete.id_collar = id_local;
    paquete.bpm = pulsaciones;
    paquete.latitud = (uint32_t)(gps_latitud*10000) // un 0 por cada decimal, lo tengo q pasar a numero entero > ej -420690
    // *etc
    */
    
    lora_send_packetb((uint8_t*)&paquete,sizeof(payload_t)); // envio el paquete, que con este formato son 13 bytes
}

void lora_init(void){
    // Configurar pines CS y RST como salida
    gpio_reset_pin(LORA_RESET);
    gpio_set_direction(LORA_RESET, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LORA_CS);
    lora_reset();

    // inicializacion de SPI_LoRa
    lora_spi_init();

    uint8_t version = lora_read_register(0x42);
    ESP_LOGI(TAG, "LoRa version register: 0x%02X", version);

    // Configuración básica LoRa (modo standby, frecuencia, potencia, etc.)
    lora_write_register(0x01, 0x80); // RegOpMode: LoRa + Sleep
    vTaskDelay(pdMS_TO_TICKS(10));
    lora_write_register(0x01, 0x81); // RegOpMode: LoRa + standby

    // Frecuencia 433 MHz (para SX1278)
    lora_write_register(0x06, 0x6C);
    lora_write_register(0x07, 0x80);
    lora_write_register(0x08, 0x00);

    // Potencia de transmisión
    lora_write_register(0x09, 0x8F); // Potencia supuestamente "ideal"
}