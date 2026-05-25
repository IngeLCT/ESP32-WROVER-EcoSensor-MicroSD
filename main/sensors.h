#pragma once
#include "esp_err.h"
#include "captive_manager.h"
#include <stdint.h>
#include <stddef.h>

extern const float ECO_SCD40_TEMP_OFFSET_C;
extern const float ECO_SEN55_TEMP_OFFSET_C;

typedef struct {
    // SCD4x
    uint16_t co2;
    float scd_temp;
    float scd_hum;

    // SEN5x
    float pm1p0;
    float pm2p5;
    float pm4p0;
    float pm10p0;
    float voc;
    float nox;
    float sen_temp;
    float sen_hum;

    // Derivados
    float avg_temp;
    float avg_hum;
} SensorData;

// Códigos de diagnóstico
typedef enum {
    SENSOR_DIAG_OK           = 0,
    SENSOR_DIAG_CRC          = 1,
    SENSOR_DIAG_TIMEOUT      = 2,
    SENSOR_DIAG_OUT_OF_RANGE = 3,
    SENSOR_DIAG_I2C_TX       = 4,
    SENSOR_DIAG_I2C_RX       = 5,
    SENSOR_DIAG_CO2_ZERO     = 6,
    SENSOR_DIAG_CO2_TOO_HIGH = 7,
    SENSOR_DIAG_NOT_READY    = 8,
    SENSOR_DIAG_OTHER        = 99
} sensor_diag_code_t;

esp_err_t sensors_init_all(void);
esp_err_t sensors_read_scd40(SensorData *out);
esp_err_t sensors_read_sen55(SensorData *out);
esp_err_t sensors_read(SensorData *out);

// JSON local simplificado para monitor/depuración
void sensors_format_json(const SensorData *d, char *buf, size_t buf_size);

int sensors_get_last_scd40_diag(void);
int sensors_get_last_sen55_diag(void);
uint16_t sensors_get_last_scd40_raw_co2(void);
uint16_t sensors_get_last_scd40_raw_temp(void);
uint16_t sensors_get_last_scd40_raw_hum(void);
float sensors_get_last_scd40_temp(void);
float sensors_get_last_scd40_hum(void);
float sensors_get_last_sen55_temp(void);
float sensors_get_last_sen55_hum(void);
bool sensors_get_scd40_temperature_offset(float *offset_c, uint16_t *raw_offset);
bool sensors_get_sen55_temperature_offset(float *offset_c, int16_t *raw_offset);
const char *sensors_get_last_scd40_raw_bytes(void);
const char *sensors_get_last_scd40_error(void);
void sensors_reset_diag(void);
