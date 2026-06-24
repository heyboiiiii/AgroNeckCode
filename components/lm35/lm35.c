#include "lm35.h"
#include "esp_log.h"

static const char *TAG = "LM35";
static adc_oneshot_unit_handle_t adc_handle = NULL;

esp_err_t lm35_init(void) {
    ESP_LOGI(TAG, "Inicializando LM35...");
    
    // Configurar el ADC oneshot
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = LM35_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configurar el canal ADC
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = LM35_ATTENUATION,
    };
    
    ret = adc_oneshot_config_channel(adc_handle, LM35_ADC_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando canal ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "LM35 inicializado correctamente");
    return ESP_OK;
}

esp_err_t lm35_read(lm35_data_t *data) {
    if (adc_handle == NULL) {
        ESP_LOGE(TAG, "ADC no inicializado");
        return ESP_ERR_INVALID_STATE;
    }
    
    int adc_raw = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, LM35_ADC_CHANNEL, &adc_raw);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Convertir valor ADC a voltaje (12 bits, rango 0-3.3V)
    // voltage (mV) = (adc_raw / 4095) * 3300
    float voltage_mv = (float)adc_raw * 3300.0f / 4095.0f;
    
    // LM35: 10mV por grado Celsius
    // temperatura (°C) = voltage_mv (mV) / 10
    data->lm_amb_temp = voltage_mv / 10.0f;
    
    ESP_LOGI(TAG, "ADC: %d | Voltaje: %.2f mV | Temperatura: %.2f °C", 
             adc_raw, voltage_mv, data->lm_amb_temp);
    
    return ESP_OK;
}

void lm35_deinit(void) {
    if (adc_handle != NULL) {
        adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
        ESP_LOGI(TAG, "LM35 desinicializado");
    }
}
