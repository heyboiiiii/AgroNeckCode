#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

// Configuración del pin ADC
#define LM35_ADC_UNIT           ADC_UNIT_1
#define LM35_ADC_CHANNEL        ADC_CHANNEL_6  // GPIO34
#define LM35_ATTENUATION        ADC_ATTEN_DB_12  // 0-3.3V

// Estructura para almacenar datos del LM35
typedef struct {
    float lm_amb_temp;  // Temperatura ambiente en °C
} lm35_data_t;

// Funciones
esp_err_t lm35_init(void);
esp_err_t lm35_read(lm35_data_t *data);
void lm35_deinit(void);