#include "neo6m.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"    
#include "freertos/FreeRTOS.h"

#define BUFFER (1024)
char buf[BUFFER];

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
    };
    
    uart_param_config(uart_num, &uart_config);
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUFFER, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, NEO6M_RX, NEO6M_TX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI("gps_starting","Finalizo la funcion gps_starting");
}

void raw_nmea(double *latitude, double *longitude,char *lat_hemisphere, char *lon_hemisphere, float *speedKmh)
{
    ESP_LOGI("raw_nmea", "Funcion invocada, leyendo bytes del uart.");
    memset(buf, 0, BUFFER);
    uart_read_bytes(UART_NUM_1, buf, BUFFER, portMAX_DELAY);
    //ESP_LOGI(TAG, "Data: %s", buf);
    ESP_LOGI("raw_nmea", "Termino de leer los bytes del uart, procesando datos.");
    //data adquisiton latitude and longtitude
    char lat[12], lon[12];
    // Declare these new char variables to store 'S', 'W', 'N', or 'E'
    char lat_h; // Will store 'S' or 'N'
    char lon_h; // Will store 'W' or 'E'
    
    const char *indexGP;

    indexGP = strstr(buf, "$GPGGA");
    
    if (indexGP != NULL) {
        
        sscanf(indexGP, "$GPGGA,%*[^,],%11[^,],%c,%11[^,],%c", lat, &lat_h, lon, &lon_h);

        *latitude = convert_to_decimal_degrees(atof(lat));
        *longitude = convert_to_decimal_degrees(atof(lon));

        *lat_hemisphere = lat_h;
        *lon_hemisphere = lon_h;

        //ESP_LOGI(TAG,"lat: %lf / %c ^^ lon; %lf / %c",latitude,lat_hemisphere,longitude,lon_hemisphere);       
    }

    indexGP = strstr(buf, "$GPRMC");

    if(indexGP != NULL){
        *speedKmh = parse_gprmc_speed(indexGP);
    }
}