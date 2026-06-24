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

void lora_spi_init() {
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
        .clock_speed_hz = 10*1000*1000,
        .mode = 0,
        .spics_io_num = LORA_CS,
        .queue_size = 7,
    };
    
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &lora_spi));
}

// Función para reiniciar el módulo
void lora_reset() {
    gpio_set_level(LORA_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

uint8_t lora_read_register(uint8_t reg) {
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
void lora_write_register(uint8_t address, uint8_t payload){
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
void lora_send_packet(const char *data) {
    int length = strlen(data);
    // Set payload length --> (register 0x22)
    lora_write_register(0x22, length);

    // FIFO TX base address
    lora_write_register(0x0E, 0x00);
    lora_write_register(0x0D, 0x00); // FIFO addr ptr

    // Cargar datos en FIFO
    for (int i = 0; i < length; i++) {
        lora_write_register(0x00, data[i]);
    }
    //limpiar flags IRQ antes de tx
    lora_write_register(0x12,0xFF);

    // Cambiar a modo TX
    lora_write_register(0x01, 0x83); // RegOpMode = TX

    uint8_t irq_flags;
    int timeout = 1000; // simple timeout to avoid infinite loop
    
    while (timeout--) {
        irq_flags = lora_read_register(0x12);
        if (irq_flags & 0x08) { // TxDone
            ESP_LOGI(TAG, "Mensaje cargado en FIFO: %s", data);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Clear IRQ flags after transmission
    lora_write_register(0x12, 0xFF);
}

void lora_init(){
    // Configurar pines CS y RST como salida
    gpio_reset_pin(LORA_RESET);
    gpio_set_direction(LORA_RESET, GPIO_MODE_OUTPUT);
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