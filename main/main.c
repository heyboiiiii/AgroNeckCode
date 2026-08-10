// DEPENDENCIAS
#include <stdio.h>
#include "esp_log.h"    
#include "esp_sleep.h"
#include "esp_adc/adc_oneshot.h" // Librería para el ADC
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

#include "lora.h"       // Librería para el módulo LoRa
#include "neo6m.h"      // Librería para el módulo GPS Neo-6MV2
#include "mpumlx.h"    // Librería para el sensor de movimiento MPU6050
#include "lm35.h"      // Librería para el sensor de temperatura LM35

#define Vout_LM35       34 // GPIO de Salida del sensor de temperatura LM35
#define H_COEFICIENTE   0.18f // Constante de acoplamiento térmico para el pelaje (Calibrar)

#define WAKEUP_GPIO     27 // GPIO que despierta del modo sleep 
#define WAKEUP_LEVEL    1 // Activarse con nivel alto/flanco ascendente

// GPIOS de MOSFETS (Control Placa Solaria)
// C1+C4 o C2+C3
#define MOSFET1 25 // GPIO del MOSFET CONTROL3
#define MOSFET2 26 // GPIO del MOSFET CONTROL4
#define MOSFET3 32 // GPIO del MOSFET CONTROL1
#define MOSFET4 33 // GPIO del MOSFET CONTROL2

#define GPIO_MOSFET_MASK ((1ULL<<MOSFET1) | (1ULL<<MOSFET2) | (1ULL<<MOSFET3) | (1ULL<<MOSFET4)) // creo una mascara para manejar los 4 pines (no hay ganas de hacer 1x1)

RTC_DATA_ATTR bool estado_mosfet = true; // variable para seguir el estado de los mosfets
// true = estado inicial, 2+3
// false = estado secundario, 1+4

// GpsVars
double latitude; double longitude; char lat_hemisphere; char lon_hemisphere; float velocidad;

// TempVars
float lm_amb_temp;  // Temperatura ambiente del LM35
mlx90614_data_t mlx_data; // Temperatura piel del animal


// inicializo salidas p/ mosfets
void init_mosfet_gpios() {
    gpio_hold_dis(MOSFET1);
    gpio_hold_dis(MOSFET2);
    gpio_hold_dis(MOSFET3);
    gpio_hold_dis(MOSFET4);
    gpio_config_t io_config = {
        .intr_type = GPIO_INTR_DISABLE,           // Deshabilitar interrupciones
        .mode = GPIO_MODE_OUTPUT,                 // Configurar como salida
        .pin_bit_mask = GPIO_MOSFET_MASK,         // Uso la mascara
    };
    gpio_config(&io_config);
}

void switch_mosfet() {
    estado_mosfet = !estado_mosfet; // invierto estado de los mosfets
    if (estado_mosfet) { // estado inicial
        gpio_set_level(MOSFET1,0); // desactivo 1+4
        gpio_set_level(MOSFET4,0);
        gpio_set_level(MOSFET2,1); // prendo 2+3
        gpio_set_level(MOSFET3,1);
    } else { // estado secundario
        gpio_set_level(MOSFET2,0); // desactivo 2+3
        gpio_set_level(MOSFET3,0);
        gpio_set_level(MOSFET1,1); // prendo 1+4
        gpio_set_level(MOSFET4,1);
    }
}

// Funciones de lectura de sensores
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

// Función para calcular la temperatura interna de la vaca
void internal_temp() {
    ESP_LOGI("Temp.Calc","Calculando temperatura interna...");
    // Calcular la temperatura interna de la vaca con la fórmula: T_interna = T_ambiente + (T_objeto - T_ambiente) * H_COEFICIENTE
    float temp_interna = lm_amb_temp + (mlx_data.mlx_object_temp - lm_amb_temp) * H_COEFICIENTE;
    ESP_LOGI("Temp.Calc","Temperatura interna estimada: %.2f °C", temp_interna); 
}



// Función despertar
void sensar_enviar(void *pvParameters){ // funcionar para sensar y enviar todos los datos al despertar del deep sleep
    // inicializar buses y perifericos
    gps_starting();
    init_i2c();
    mpu6050_init(I2C_NUM_0);
    lm35_init();
    init_mosfet_gpios();

    // lectura de sensores
    read_gps();
    read_mpu6050();
    read_mlx90614();
    read_lm35();
    internal_temp();

    // payload_t paquete; // creo un paquete
    // paquete.latitud = (int32_t)(gps_latitud * 10000.0); // cargo los datos, en este caso solo estoy cargando datos gps como ejemplo
    // paquete.longitud = (int32_t)(gps_longitud * 10000.0);

    // transmitir_datos(&paquete);

    // limpia el pin de int para volver a dormir
    mpu6050_clear_int(I2C_NUM_0); 

    // congelo los mosfets antes de irme a dormir
    gpio_hold_en(MOSFET1);
    gpio_hold_en(MOSFET2);
    gpio_hold_en(MOSFET3);
    gpio_hold_en(MOSFET4);

    // creo una interrupcion de despertado
    esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO,WAKEUP_LEVEL);

    vTaskDelay(pdMS_TO_TICKS(100));

    // entro/vuelvo a deep sleep
    esp_deep_sleep_start();

    vTaskDelete(NULL);
};

// MAIN
void app_main(void)
{
    ESP_LOGI("MAIN","Comenzando los procesos principales");
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_EXT0) { // me desperte por alguna razon distinta que una interrupcion, como primer encendido
        init_i2c(); //inicializa i2c
        mpu6050_init(I2C_NUM_0); //inicializa mpu6050
        mpu6050_enable_wom(I2C_NUM_0, MPU6050_THRESHOLD); //inicializa el Wake-On-Motion del mpu6050 
        mpu6050_clear_int(I2C_NUM_0); //limpia el pin de int de mpu6050 (por las dudas)
        esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, WAKEUP_LEVEL); //configura quien puede despertarlo
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_deep_sleep_start(); //vuelve a dormir
        return;
    }

    // si llego hasta aca, fue pq me desperte por interrupcion
    xTaskCreate(sensar_enviar,"sensar_enviar_task",4096,NULL,5,NULL);    
}