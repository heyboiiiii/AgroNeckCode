// DEPENDENCIAS
#include <stdio.h>
#include <sys/time.h>
#include "esp_log.h"    
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_adc/adc_oneshot.h" 
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>

#include "lora.h"       // Librería para el módulo LoRa
#include "neo6m.h"      // Librería para el módulo GPS NEO-6MV2
#include "mpumlx.h"    // Librería para el sensor de movimiento MPU6050
#include "lm35.h"      // Librería para el sensor de temperatura LM35

// MODO SLEEP
#define WAKEUP_GPIO     27 // GPIO que despierta del modo sleep 
#define WAKEUP_LEVEL    1 // Activarse con nivel alto/flanco ascendente

// CONTROL DE ALIMENTACIÓN - C1+C4 o C2+C3
#define MOSFET1 25 // GPIO del MOSFET CONTROL3
#define MOSFET2 26 // GPIO del MOSFET CONTROL4
#define MOSFET3 32 // GPIO del MOSFET CONTROL1
#define MOSFET4 33 // GPIO del MOSFET CONTROL2
// Mascara de manejo de los 4 pines
#define GPIO_MOSFET_MASK ((1ULL<<MOSFET1) | (1ULL<<MOSFET2) | (1ULL<<MOSFET3) | (1ULL<<MOSFET4))
// Variable que guarda el estado de mosfets aun en Deep Sleep
RTC_DATA_ATTR bool estado_mosfet = true; //true = estado inicial, 2+3 y false = estado secundario, 1+4
// Conmutación de baterías
#define TIEMPO_CAMBIO_BATS 10ULL 
RTC_DATA_ATTR uint64_t tiempo_switch = 0;
RTC_DATA_ATTR bool sistema_inicializado = false;


// VARIABLES DE DATOS
// SISTEMA DE POSICION
double latitude; double longitude; char lat_hemisphere; char lon_hemisphere; float velocidad;

// SISTEMA DE TEMPERATURA
#define Vout_LM35       34 // GPIO de Salida del sensor de temperatura LM35
#define H_COEFICIENTE   0.18f // Constante de acoplamiento térmico para el pelaje (Calibrar)
float lm_amb_temp;  // Temperatura ambiente del LM35
mlx90614_data_t mlx_data; // Temperatura piel del animal
float temp_interna; // temperatura final del calculo

// Inicialización de Salida de MOSFETs
void init_mosfet_gpios() {
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(MOSFET1);
    gpio_hold_dis(MOSFET2);
    gpio_hold_dis(MOSFET3);
    gpio_hold_dis(MOSFET4);
    gpio_config_t io_config = {
        .intr_type = GPIO_INTR_DISABLE,           // Deshabilitar interrupciones
        .mode = GPIO_MODE_OUTPUT,                 // Configurar como salida
        .pin_bit_mask = GPIO_MOSFET_MASK,         // Uso la mascara
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_config);
}

// Declara los estados de los MOSFETs
void aplica_estado_mosfet() {
    ESP_LOGI("MOSFETS","Aplicando estado %d...",estado_mosfet);
    if (estado_mosfet) { // Estado inicial: 2+3
        gpio_set_level(MOSFET1, 0);
        gpio_set_level(MOSFET4, 0);
        gpio_set_level(MOSFET2, 1);
        gpio_set_level(MOSFET3, 1);
    } else { // Estado secundario: 1+4
        gpio_set_level(MOSFET2, 0);
        gpio_set_level(MOSFET3, 0);
        gpio_set_level(MOSFET1, 1);
        gpio_set_level(MOSFET4, 1);
    }
}

// Switcheo de MOSFETs (alternar entre baterías)
void switch_mosfet() {
    ESP_LOGI("MOSFETS","Switcheando mosfets...");
    estado_mosfet = !estado_mosfet; 
    aplica_estado_mosfet();
}

uint64_t tiempo_medido(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec;
}


void hora_actual() {
    uint64_t tiempo_actual = tiempo_medido();

    // Si es el primer encendido físico del equipo
    if (!sistema_inicializado) {
        tiempo_switch = tiempo_actual;
        sistema_inicializado = true;
        aplica_estado_mosfet();
        ESP_LOGI("MAIN", "Tiempo base: %llu", tiempo_switch);
        return;
    }

    uint64_t transcurrido = tiempo_actual - tiempo_switch;
    ESP_LOGI("MOSFETS", "Tiempo actual: %llu s, ultimo switch: %llu, transcurrido: %llu s", 
             tiempo_actual, tiempo_switch, transcurrido);

    // Evalúa si ya pasó el tiempo necesario para cambiar de batería
    if (transcurrido >= TIEMPO_CAMBIO_BATS) {
        switch_mosfet();
        tiempo_switch = tiempo_actual; // Se actualiza el tiempo base con la hora actual
    } else {
        ESP_LOGI("MOSFETS", "Aún no transcurrió el tiempo. Se mantiene estado actual.");
        aplica_estado_mosfet();
    }
}


/*
    FUNCIONES DE LECTURA DE SENSORES
*/


// Lectura de Temperatura LM35
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

// Lectura de Posición NEO-6MV2
void read_gps() {
    ESP_LOGI("GPS","Intentando leer GPS...");
    raw_nmea(&latitude,&longitude,&lat_hemisphere,&lon_hemisphere,&velocidad);
    ESP_LOGI("GPS","Latitud: %f %c, Longitud: %f %c", latitude, lat_hemisphere, longitude, lon_hemisphere);
}

// Lectura de Aceleración y Giroscopio MPU6050
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

// Lectura de Temperatura MLX90614
void read_mlx90614() {
    ESP_LOGI("MLX90614","Intentando leer MLX90614...");
    esp_err_t ret = mlx90614_read(&mlx_data, I2C_NUM_0);
    if (ret == ESP_OK) {
        ESP_LOGI("MLX90614","Temperatura objeto: %.2f °C", mlx_data.mlx_object_temp);
    } else {
        ESP_LOGE("MLX90614","Error leyendo datos: %s", esp_err_to_name(ret));
    }
    }


/*
    FUNCIONES DE CALCULO Y PROCESAMIENTO DE DATOS
*/


// Cálculo de Temperatura Interna del Ganado
void internal_temp() {
    ESP_LOGI("Temp.Calc","Calculando temperatura interna...");
    // Fórmula: T_interna = T_ambiente + (T_objeto - T_ambiente) * H_COEFICIENTE
    temp_interna = (lm_amb_temp+8) + (mlx_data.mlx_object_temp - (lm_amb_temp+8)) * H_COEFICIENTE;
    ESP_LOGI("Temp.Calc","Temperatura interna estimada: %.2f °C", temp_interna); 
}

// Inicialización, Lectura y Procesamiento de Datos al despertar
void sensar_enviar(void *pvParameters){ 
    // Inicialización de buses y perifericos
    ESP_LOGI("func_sensar_enviar","Iniciando buses y perifericos.");
    gps_starting();
    init_i2c();
    mpu6050_init(I2C_NUM_0);
    lm35_init();
    init_mosfet_gpios();

    ESP_LOGI("func_sensar_enviar","Leyendo todos los sensores...");
    // Lectura de sensores
    read_gps(); // variables latitude , longitude
    read_mpu6050(); // nada 
    read_mlx90614(); // no necesito ninguna variable
    read_lm35(); // no necesito ninguna variable
    internal_temp(); // uso variable temp_interna

    ESP_LOGI("func_sensar_enviar","Empaquetando y enviando...");
    payload_t paquete; // creo un paquete
    // paquete.id_collar = 0; // falta un modo de configurar el id para cada collar
    paquete.latitud = (int32_t)(latitude * 1000000.0); // cargo los datos, en este caso solo estoy cargando datos gps como ejemplo
    paquete.longitud = (int32_t)(longitude * 1000000.0);
    paquete.temperatura = temp_interna;

    transmitir_datos(&paquete);

    // Realizo tareas necesarias de MOSFETs
    ESP_LOGI("func_sensar_enviar","Congelando MOSFETs...");
    hora_actual();
    gpio_hold_en(MOSFET1);
    gpio_hold_en(MOSFET2);
    gpio_hold_en(MOSFET3);
    gpio_hold_en(MOSFET4);

    // Limpia el pin de INT previo al Deep Sleep
    ESP_LOGI("func_sensar_enviar","Limpiando INT MPU.");
    mpu6050_clear_int(I2C_NUM_0); 

    ESP_LOGI("func_sensar_enviar","Preparando interrupción antes de ir a dormir...");
    // Interrupción de despertado
    esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO,WAKEUP_LEVEL);

    ESP_LOGI("func_sensar_enviar","Entrando en deep sleep...");
    vTaskDelay(pdMS_TO_TICKS(100));

    // Entra en estado Deep Sleep
    esp_deep_sleep_start();

    vTaskDelete(NULL);
};

// MAIN
void app_main(void)
{
    ESP_LOGI("MAIN","Comenzando los procesos principales");
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    // Despierto por razones que no son Wake-On-Motion de MPU6050
    if (cause != ESP_SLEEP_WAKEUP_EXT0) { // MAIN_fwu = main first wake up
        //Inicialización Necesaria para activar el WOM 
        ESP_LOGI("MAIN_fwu","Primer arranque, inicializando buses.");
        init_i2c(); 
        mpu6050_init(I2C_NUM_0);
        mpu6050_enable_wom(I2C_NUM_0, MPU6050_THRESHOLD); 
        mpu6050_clear_int(I2C_NUM_0); // No necesario, pero para evitar problemas

        ESP_LOGI("MAIN_fwu","Inicio y congelo los mosfets.");
        //Configuro MOSFETs antes de entrar en Deep Sleep
        init_mosfet_gpios();
        hora_actual();
        aplica_estado_mosfet();
        gpio_hold_en(MOSFET1);
        gpio_hold_en(MOSFET2);
        gpio_hold_en(MOSFET3);
        gpio_hold_en(MOSFET4);
        
        //Configuración de GPIO que despierte a la ESP32
        ESP_LOGI("MAIN_fwu","Preparo interrupción.");
        esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, WAKEUP_LEVEL); 
        
        ESP_LOGI("MAIN_fwu","Entrando en deep sleep...");
        vTaskDelay(pdMS_TO_TICKS(100));
        
        //Entra en estado Deep Sleep
        esp_deep_sleep_start(); 
        return;
    }

    // Despierto de Deep Sleep por Wake-On-Motion de MPU6050
    xTaskCreate(sensar_enviar,"sensar_enviar_task",4096,NULL,5,NULL);    
}