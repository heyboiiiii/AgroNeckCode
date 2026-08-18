#include "mpumlx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPUMLX";
// handles para buses
static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t mpu_dev_handle = NULL;
static i2c_master_dev_handle_t mlx_dev_handle = NULL;

//I2C
void init_i2c() {
// 1. Configuro bus i2c master
    ESP_LOGI(TAG, "Configurando I2C...");
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = MLX_MPU6050_SDA,
        .scl_io_num = MLX_MPU6050_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle)); 

    // inicializo mpu y mlx
    i2c_device_config_t mpu_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu_dev_cfg, &mpu_dev_handle));

    i2c_device_config_t mlx_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MLX90614_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mlx_dev_cfg, &mlx_dev_handle));
}

/*
// MPU6050
esp_err_t mpu6050_write_byte(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(i2c_num, MPU6050_I2C_ADDR, write_buf, sizeof(write_buf), 100);
}

esp_err_t mpu6050_read_bytes(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(i2c_num, MPU6050_I2C_ADDR, &reg_addr, 1, data, len, 100);
}

esp_err_t mpu6050_init(i2c_port_t i2c_num) {
    uint8_t who_am_i;
    esp_err_t ret = mpu6050_read_bytes(I2C_NUM_0, MPU6050_WHO_AM_I_REG, &who_am_i, 1);

    if (ret == ESP_OK) {
        // SI ES 0x68 O 0x70, ES CORRECTO
        if (who_am_i == 0x68 || who_am_i == 0x70) {
            ESP_LOGI(TAG, "MPU Detectado con exito. WHO_AM_I: 0x%02X", who_am_i);
        } else {
            // SOLO ENTRA ACÁ SI LEE OTRO VALOR RARO (Como 0x00 o 0xFF)
            ESP_LOGE(TAG, "WHO_AM_I incorrecto (Leido: 0x%02X, Esperado: 0x68 o 0x70)", who_am_i);
            if (who_am_i == 0x00 || who_am_i == 0xFF) {
                ESP_LOGE(TAG, "Posible fallo de conexion o alimentacion");
            }
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "Fallo al leer WHO_AM_I: %s", esp_err_to_name(ret));
        return ret;
    }

    // Sacar del modo sleep
    uint8_t cmd[] = { MPU6050_PWR_MGMT_1_REG, 0x00 };
    ret = i2c_master_write_to_device(i2c_num, MPU6050_I2C_ADDR, cmd, 2, 100);
    return ret;
}

esp_err_t mpu6050_read(mpu6050_data_t *data, i2c_port_t i2c_num) {
    uint8_t buffer[14];
    esp_err_t ret = mpu6050_read_bytes(i2c_num, MPU6050_ACCEL_XOUT_H, buffer, 14);

    if(ret != ESP_OK){
        ESP_LOGE(TAG, "Error de lectura %d", ret);
        return ret;
    }

    data->ax = (int16_t)(buffer[0] << 8 | buffer[1]);
    data->ay = (int16_t)(buffer[2] << 8 | buffer[3]);
    data->az = (int16_t)(buffer[4] << 8 | buffer[5]);

    data->gx = (int16_t)(buffer[8] << 8 | buffer[9]);
    data->gy = (int16_t)(buffer[10] << 8 | buffer[11]);
    data->gz = (int16_t)(buffer[12] << 8 | buffer[13]);


    // Mostrar valores en el monitor serial
    ESP_LOGI(TAG, "Acelerómetro - X: %d, Y: %d, Z: %d", data->ax, data->ay, data->az);
    ESP_LOGI(TAG, "Giroscopio - X: %d, Y: %d, Z: %d", data->gx, data->gy, data->gz);
    return ESP_OK;
}


    // Wake-On-Motion de MPU-6050
    //     MPU6050_ACCEL_CONFIG: Configura el DHPS (Digital High Pass Filter).
    //     MPU6050_MOT_THR: Umbral mínimo para activarse por WOM.
    //     MPU6050_MOT_DUR: Duración necesaria del umbral.
    //     MPU6050_INT_PIN_CFG: Se configura el latch hasta finalizar la interrupción.
    //     MPU6050_INT_ENABLE: Permite enviar un HIGH al INT en caso de activarse por Motion.
    //     MPU6050_PWR_MGMT_1_REG: Utiliza los pines SLEEP, CYCLE y TEMP_DIS para configurar el modo Only Low Power.
    //     MPU6050_PWR_MGMT_2_REG: Similar al 1, activa STBY_XG, STBY_YG y STBY_ZG para configurar el modo Only Low Power.

esp_err_t mpu6050_enable_wom(i2c_port_t i2c_num, uint8_t threshold_val) {
    esp_err_t ret;
    ret = mpu6050_write_byte(i2c_num, MPU6050_ACCEL_CONFIG, 0x01);
    if (ret != ESP_OK) return ret;
    ret = mpu6050_write_byte(i2c_num, MPU6050_MOT_THR, threshold_val);
    if (ret != ESP_OK) return ret;
    ret = mpu6050_write_byte(i2c_num, MPU6050_MOT_DUR, 0x01);
    if (ret != ESP_OK) return ret;
    ret = mpu6050_write_byte(i2c_num, MPU6050_INT_PIN_CFG, 0x20);
    if (ret != ESP_OK) return ret;
    ret = mpu6050_write_byte(i2c_num, MPU6050_INT_ENABLE, 0x40);
    if (ret != ESP_OK) return ret;
    ret = mpu6050_write_byte(i2c_num, MPU6050_PWR_MGMT_1_REG, 0x28);
    if (ret != ESP_OK) return ret;
    ret = mpu6050_write_byte(i2c_num, MPU6050_PWR_MGMT_2_REG, 0x07);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

//leer int_status, lo que genera un 0 en el pin int
esp_err_t mpu6050_clear_int(i2c_port_t i2c_num) {
    uint8_t status;
    esp_err_t ret = mpu6050_read_bytes(i2c_num, MPU6050_INT_STATUS, &status, 1);
    return ret;
}
*/
// MLX90614
esp_err_t mlx90614_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(mlx_dev_handle, write_buf, sizeof(write_buf), 100);
}

esp_err_t mlx90614_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(mlx_dev_handle, &reg_addr, 1, data, len, 100);
}

esp_err_t mlx90614_read(mlx90614_data_t *data) {
    uint8_t buffer[3];
    esp_err_t ret = mlx90614_read_bytes(0x07, buffer, 3); // Leer temperatura  (0x07 = objeto | 0x06 = ambiente)
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo temperatura de la superficie: %s", esp_err_to_name(ret));
        return ret;
    }
    uint16_t temp_sup_raw = (buffer[1] << 8) | buffer[0];
    data->mlx_object_temp = (temp_sup_raw * 0.02) - 273.15; // Convertir a grados Celsius

    /*esp_err_t ret2 = mlx90614_read_bytes(i2c_num, 0x06, buffer, 3); // Leer temperatura ambiente
    if (ret2 != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo temperatura ambiente: %s", esp_err_to_name(ret2));
        return ret2;
    }
    uint16_t temp_amb_raw = (buffer[1] << 8) | buffer[0];
    data->mlx_amb_temp = (temp_amb_raw * 0.02) - 273.15; // Convertir a grados Celsius*/
    return ESP_OK;
}