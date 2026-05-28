// DEPENDENCIAS
#include <stdio.h>
#include "esp_log.h"    
#include "esp_adc/adc_oneshot.h" // Librería para el ADC

#include "lora.h"       // Librería para el módulo LoRa
#include "neo6m.h"      // Librería para el módulo GPS Neo-6MV2
#include "mpumlx.h"    // Librería para el sensor de movimiento MPU6050

// DEFINICIONES 

#define Vout_LN35       34 // Salida del sensor de temperatura LN35
#define Vout_PE         35 // Salida del piezoeléctrico

// VARIABLES
// GPS
double latitude; double longitude; char lat_hemisphere; char lon_hemisphere; float velocidad;

// ADC
adc_oneshot_unit_handle_t adc1;

// FUNCIONES
void read_gps() {
    ESP_LOGI("read_gps","Intentando leer GPS...");
    raw_nmea(&latitude,&longitude,&lat_hemisphere,&lon_hemisphere,&velocidad);
    ESP_LOGI("GPS","Latitud: %f %c, Longitud: %f %c", latitude, lat_hemisphere, longitude, lon_hemisphere);
    ESP_LOGI("read_gps","Se intento.");
}

void read_mpu6050() {
    ESP_LOGI("read_mpu6050","Intentando leer MPU6050...");
    mpu6050_data_t data;
    esp_err_t ret = mpu6050_read(&data, I2C_NUM_0);
    if (ret == ESP_OK) {
        ESP_LOGI("MPU6050","Aceleración - X: %d, Y: %d, Z: %d", data.ax, data.ay, data.az);
        ESP_LOGI("MPU6050","Giroscopio - X: %d, Y: %d, Z: %d", data.gx, data.gy, data.gz);
    } else {
        ESP_LOGE("MPU6050","Error leyendo datos: %s", esp_err_to_name(ret));
    }
}

void read_mlx90614() {
    ESP_LOGI("read_mlx90614","Intentando leer MLX90614...");
    mlx90614_data_t data;
    esp_err_t ret = mlx90614_read(&data, I2C_NUM_0);
    if (ret == ESP_OK) {
        ESP_LOGI("MLX90614","Temperatura ambiente: %.2f °C, Temperatura objeto: %.2f °C", data.ambient_temp, data.object_temp);
    } else {
        ESP_LOGE("MLX90614","Error leyendo datos: %s", esp_err_to_name(ret));
    }
}

void init_adc() {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    
    adc_oneshot_new_unit(&init_config, &adc1);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };

    adc_oneshot_config_channel(
        adc1,
        ADC_CHANNEL_6,
        &config
    );
}

void read_ln35() {
    int raw_value = 0;
    adc_oneshot_read(adc1,ADC_CHANNEL_6, &raw_value); // Leer valor crudo del ADC
    float voltage = raw_value * (3.3 / 4095.0); // Convertir a voltaje
    float temperature = voltage * 100.0; // Convertir a temperatura (LN35 tiene una sensibilidad de 10mV/°C)
    ESP_LOGI("LN35","Temperatura: %.2f °C", temperature);
    ESP_LOGI("LN35","Valor crudo: %d, Voltaje: %.2f V", raw_value, voltage);
}

// MAIN
void app_main(void)
{
    ESP_LOGI("MAIN","Hola! Empezamos.");
    // gps_starting(); // Inicializa el GPS
    init_i2c(); // inicializa el bus I2C para el MPU6050 y el MLX90614
    init_adc(); // Inicializa el ADC para el LN35
    while (1) {
        //read_mpu6050();
        read_mlx90614();
        ESP_LOGW("-","--------------------------------------------------------------");
        read_ln35();
        vTaskDelay(pdMS_TO_TICKS(500));    
    }
}