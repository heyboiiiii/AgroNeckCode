// Pines GPS
#define NEO6M_RX        16
#define NEO6M_TX        17

// Funciones
void gps_starting(void);
void raw_nmea(double *latitude, double *longitude, char *lat_hemisphere, char *lon_hemisphere, float *speedKmh);