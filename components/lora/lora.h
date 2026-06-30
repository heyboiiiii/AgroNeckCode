#include <stdint.h>

// Pines LORA
#define LORA_CS         5
#define LORA_RESET      14
#define LORA_CLK        18
#define LORA_MISO       19
#define LORA_MOSI       23
#define LORA_DIO0       26

// Estructuras
typedef struct {
    uint16_t id_collar;
    uint8_t bpm;
    int16_t temperatura;
    int32_t latitud;
    int32_t longitud;
} __attribute__((packed)) payload_t;

// Funciones
void lora_init();
void lora_spi_init();
void lora_reset();
uint8_t lora_read_register(uint8_t reg);
void lora_write_register(uint8_t adress, uint8_t payload);
void lora_send_packetb(const char *data); // lora_send_packetb(inary)