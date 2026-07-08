// DEPENDENCIAS
#include <stdio.h>
#include "esp_log.h"    
#include "esp_adc/adc_oneshot.h" // Librería para el ADC

#include "lora.h"       // Librería para el módulo LoRa
#include "neo6m.h"      // Librería para el módulo GPS Neo-6MV2
#include "mpumlx.h"    // Librería para el sensor de movimiento MPU6050
#include "lm35.h"      // Librería para el sensor de temperatura LM35

// DEFINICIONES 

#define Vout_LM35       34 // GPIO de Salida del sensor de temperatura LM35
#define Vout_PE         35 // GPIO de Salida del piezoeléctrico
#define H_COEFICIENTE   0.18f // Constante de acoplamiento térmico para el pelaje (Calibrar)

// VARIABLES GPS
double latitude; double longitude; char lat_hemisphere; char lon_hemisphere; float velocidad;

// TEMP
float lm_amb_temp;  // Temperatura ambiente del LM35
mlx90614_data_t mlx_data; // Temperatura piel del animal

//GPIOS de MOSFETS(Control Placa Solaria):

#define MOSFET1 25 // GPIO del MOSFET 1

// FUNCIONES
void read_lm35() {
    ESP_LOGI("read_lm35","Intentando leer LM35...");
    lm35_data_t data;
    esp_err_t ret = lm35_read(&data);
    if (ret == ESP_OK) {
        lm_amb_temp = data.lm_amb_temp;
        ESP_LOGI("LM35","Temperatura: %.2f °C", lm_amb_temp);
    } else {
        ESP_LOGE("LM35","Error leyendo LM35: %s", esp_err_to_name(ret));
    }
}

void read_gps() {
    ESP_LOGI("GPS","Intentando leer GPS...");
    raw_nmea(&latitude,&longitude,&lat_hemisphere,&lon_hemisphere,&velocidad);
    ESP_LOGI("GPS","Latitud: %f %c, Longitud: %f %c", latitude, lat_hemisphere, longitude, lon_hemisphere);
}

void read_mpu6050() {
    ESP_LOGI("MPU6050","Intentando leer MPU6050...");
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
    ESP_LOGI("MLX90614","Intentando leer MLX90614...");
    esp_err_t ret = mlx90614_read(&mlx_data, I2C_NUM_0);
    if (ret == ESP_OK) {
        ESP_LOGI("MLX90614","Temperatura objeto: %.2f °C", mlx_data.mlx_object_temp);
    } else {
        ESP_LOGE("MLX90614","Error leyendo datos: %s", esp_err_to_name(ret));
    }
    }

void internal_temp() {
    ESP_LOGI("Temp.Calc","Calculando temperatura interna...");
    // Calcular la temperatura interna de la vaca con la fórmula: T_interna = T_ambiente + (T_objeto - T_ambiente) * H_COEFICIENTE
    float temp_interna = lm_amb_temp + (mlx_data.mlx_object_temp - lm_amb_temp) * H_COEFICIENTE;
    ESP_LOGI("Temp.Calc","Temperatura interna estimada: %.2f °C", temp_interna); 
}

// Logica de intercambio de baterias que alimentan al sistema y carga de las mismas.


// MAIN
void app_main(void)
{
    ESP_LOGI("MAIN","Comenzando los procesos principales");
    gps_starting(); // Inicializa el GPS
    //init_i2c(); // inicializa el bus I2C para el MPU6050 y el MLX90614
    //mpu6050_init(I2C_NUM_0);
    //lm35_init(); // Inicializa el ADC y el canal para el LM35
    while (1) {
        read_gps();
        //read_mpu6050();
        //read_mlx90614();
        //read_lm35();
        //internal_temp();
        vTaskDelay(pdMS_TO_TICKS(1500));    
    }
}