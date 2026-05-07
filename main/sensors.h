#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

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
void sensors_reset_diag(void);
