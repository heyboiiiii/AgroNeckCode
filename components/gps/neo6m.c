#include "neo6m.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"    
#include "freertos/FreeRTOS.h"

#define BUFFER_SIZE (1024)

static float parse_gprmc_speed(const char *nmea_sentence) {
    if (strstr(nmea_sentence, "$GPRMC") == NULL) return -1;

    char *tokens[12] = {0};
    char *temp = strdup(nmea_sentence);
    char *token = strtok(temp, ",");

    int i = 0;
    while (token != NULL && i < 12) {
        tokens[i++] = token;
        token = strtok(NULL, ",");
    }

    float speed_knots = 0.0;
    if (i >= 8 && tokens[7] != NULL && strlen(tokens[7]) > 0) {
        speed_knots = atof(tokens[7]);
    }

    free(temp);

    // Convert knots to km/h
    return speed_knots * 1.852;
}

static double convert_to_decimal_degrees(double raw_coordinate) {
    // Extract the integer part as degrees.
    // For 3445.30788, floor(3445.30788 / 100) gives 34.
    double degrees = floor(raw_coordinate / 100.0);

    // Extract the fractional part (after removing degrees) as minutes.
    // For 3445.30788, fmod(3445.30788, 100.0) gives 45.30788.
    double minutes = fmod(raw_coordinate, 100.0);

    // Calculate the decimal degrees.
    // 34 + (45.30788 / 60)
    return degrees + (minutes / 60.0);
}

void gps_starting()
{   
    ESP_LOGI("gps_starting", "Iniciando GPS...");
    const uart_port_t uart_num = UART_NUM_1;
    
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, NEO6M_TX, NEO6M_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUFFER_SIZE * 2, 0, 0, NULL, 0));

    ESP_LOGI("gps_starting","Finalizo la funcion gps_starting");
}

void raw_nmea(double *latitude, double *longitude,char *lat_hemisphere, char *lon_hemisphere, float *speedKmh)
{
    ESP_LOGI("func_raw_nmea","Se intenta leer GPS.");
    uint8_t buf[BUFFER_SIZE];
    
    int rxBytes = uart_read_bytes(UART_NUM_1, buf, BUFFER_SIZE - 1, pdMS_TO_TICKS(1000));
    
    if (rxBytes > 0) {
        buf[rxBytes] = '\0';
        ESP_LOGI("raw_nmea", "Bytes recibidos: %d", rxBytes);

        char lat[12] = {0}, lon[12] = {0};
        char lat_h = 0, lon_h = 0;
        
        const char *indexGP = strstr((char *)buf, "$GPGGA");
        if (indexGP != NULL) {
            if (sscanf(indexGP, "$GPGGA,%*[^,],%11[^,],%c,%11[^,],%c", lat, &lat_h, lon, &lon_h) == 4) {
                *latitude = convert_to_decimal_degrees(atof(lat));
                *longitude = convert_to_decimal_degrees(atof(lon));
                *lat_hemisphere = lat_h;
                *lon_hemisphere = lon_h;
            }
        }

        indexGP = strstr((char *)buf, "$GPRMC");
        if (indexGP != NULL) {
            *speedKmh = parse_gprmc_speed(indexGP);
        }
    } else {
        ESP_LOGW("raw_nmea", "Timeout: No se recibieron datos del UART.");
    }
}