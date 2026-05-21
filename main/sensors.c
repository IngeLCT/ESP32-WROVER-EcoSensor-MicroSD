#include "sensors.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <stdbool.h>
#include <stdio.h>

#define I2C_MASTER_SCL_IO       19
#define I2C_MASTER_SDA_IO       18
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_PORT                I2C_NUM_0

#define SCD4X_ADDR              0x62
#define SEN5X_ADDR              0x69

#define SCD40_CO2_OUTPUT_MIN    1
#define SCD40_CO2_OUTPUT_MAX    40000
#define SCD40_CO2_SPEC_MIN      400
#define SCD40_CO2_SPEC_MAX      2000

#define SEN55_READY_POLLS       30
#define SEN55_READY_DELAY_MS    20

static const char *TAG_SENS = "SENSORS";

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_scd4x_dev = NULL;
static i2c_master_dev_handle_t s_sen5x_dev = NULL;

static int s_last_scd40_diag = SENSOR_DIAG_OK;
static int s_last_sen55_diag = SENSOR_DIAG_OK;
static uint16_t s_last_scd40_raw_co2 = 0;
static uint16_t s_last_scd40_raw_temp = 0;
static uint16_t s_last_scd40_raw_hum = 0;
static float s_last_scd40_temp = 0.0f;
static float s_last_scd40_hum = 0.0f;
static char s_last_scd40_raw_bytes[28] = "";
static char s_last_scd40_error[24] = "OK";

static int map_i2c_err_to_diag(esp_err_t err, bool is_rx_stage) {
    if (err == ESP_OK) return SENSOR_DIAG_OK;
    if (err == ESP_ERR_INVALID_CRC) return SENSOR_DIAG_CRC;
    if (err == ESP_ERR_TIMEOUT) return SENSOR_DIAG_TIMEOUT;
    if (err == ESP_ERR_INVALID_RESPONSE) return SENSOR_DIAG_OUT_OF_RANGE;
    return is_rx_stage ? SENSOR_DIAG_I2C_RX : SENSOR_DIAG_I2C_TX;
}

static void scd40_set_error(const char *text) {
    snprintf(s_last_scd40_error, sizeof(s_last_scd40_error), "%s", text ? text : "UNKNOWN");
}

static void scd40_store_raw_bytes(const uint8_t *data, size_t len) {
    if (!data || len == 0) {
        s_last_scd40_raw_bytes[0] = '\0';
        return;
    }
    size_t pos = 0;
    for (size_t i = 0; i < len && pos + 3 < sizeof(s_last_scd40_raw_bytes); i++) {
        int written = snprintf(&s_last_scd40_raw_bytes[pos], sizeof(s_last_scd40_raw_bytes) - pos, "%02X%s", data[i], (i + 1 < len) ? " " : "");
        if (written <= 0) break;
        pos += (size_t)written;
    }
}

int sensors_get_last_scd40_diag(void) {
    return s_last_scd40_diag;
}

int sensors_get_last_sen55_diag(void) {
    return s_last_sen55_diag;
}

uint16_t sensors_get_last_scd40_raw_co2(void) {
    return s_last_scd40_raw_co2;
}

uint16_t sensors_get_last_scd40_raw_temp(void) {
    return s_last_scd40_raw_temp;
}

uint16_t sensors_get_last_scd40_raw_hum(void) {
    return s_last_scd40_raw_hum;
}

float sensors_get_last_scd40_temp(void) {
    return s_last_scd40_temp;
}

float sensors_get_last_scd40_hum(void) {
    return s_last_scd40_hum;
}

const char *sensors_get_last_scd40_raw_bytes(void) {
    return s_last_scd40_raw_bytes;
}

const char *sensors_get_last_scd40_error(void) {
    return s_last_scd40_error;
}

void sensors_reset_diag(void) {
    s_last_scd40_diag = SENSOR_DIAG_OK;
    s_last_sen55_diag = SENSOR_DIAG_OK;
    scd40_set_error("OK");
}

static uint8_t sensirion_crc8(const uint8_t *data, int len) {
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x31;
            else crc <<= 1;
        }
    }
    return crc;
}

static esp_err_t scd4x_start_measurement(void) {
    uint8_t cmd[2] = {0x21, 0xB1};
    return i2c_master_transmit(s_scd4x_dev, cmd, sizeof(cmd), pdMS_TO_TICKS(1000));
}

static esp_err_t scd4x_read_measurement(uint16_t *co2, float *temperature, float *humidity) {
    s_last_scd40_diag = SENSOR_DIAG_OK;
    scd40_set_error("OK");

    uint8_t cmd[2] = {0xEC, 0x05};
    esp_err_t ret = i2c_master_transmit(s_scd4x_dev, cmd, sizeof(cmd), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        s_last_scd40_diag = map_i2c_err_to_diag(ret, false);
        scd40_set_error("I2C_TX");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    uint8_t data[9];
    ret = i2c_master_receive(s_scd4x_dev, data, sizeof(data), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        s_last_scd40_diag = map_i2c_err_to_diag(ret, true);
        scd40_set_error(ret == ESP_ERR_TIMEOUT ? "TIMEOUT" : "I2C_RX");
        return ret;
    }
    scd40_store_raw_bytes(data, sizeof(data));

    if (sensirion_crc8(&data[0], 2) != data[2] ||
        sensirion_crc8(&data[3], 2) != data[5] ||
        sensirion_crc8(&data[6], 2) != data[8]) {
        ESP_LOGW(TAG_SENS, "SCD40 CRC invalido raw=%s", s_last_scd40_raw_bytes);
        s_last_scd40_diag = SENSOR_DIAG_CRC;
        scd40_set_error("CRC_INVALIDO");
        return ESP_ERR_INVALID_CRC;
    }

    *co2 = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_temp = ((uint16_t)data[3] << 8) | data[4];
    uint16_t raw_hum  = ((uint16_t)data[6] << 8) | data[7];
    s_last_scd40_raw_co2 = *co2;
    s_last_scd40_raw_temp = raw_temp;
    s_last_scd40_raw_hum = raw_hum;
    s_last_scd40_temp = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    s_last_scd40_hum = 100.0f * ((float)raw_hum / 65535.0f);

    if (*co2 == 0) {
        ESP_LOGW(TAG_SENS, "SCD40 CO2 cero raw=%s temp=%.2f hum=%.2f", s_last_scd40_raw_bytes, s_last_scd40_temp, s_last_scd40_hum);
        s_last_scd40_diag = SENSOR_DIAG_CO2_ZERO;
        scd40_set_error("CO2_ZERO");
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (*co2 > SCD40_CO2_OUTPUT_MAX) {
        ESP_LOGW(TAG_SENS, "SCD40 CO2 demasiado alto: %u ppm raw=%s", *co2, s_last_scd40_raw_bytes);
        s_last_scd40_diag = SENSOR_DIAG_CO2_TOO_HIGH;
        scd40_set_error("CO2_TOO_HIGH");
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (*co2 < SCD40_CO2_SPEC_MIN || *co2 > SCD40_CO2_SPEC_MAX) {
        ESP_LOGW(TAG_SENS,
                 "SCD40 CO2 fuera del rango especificado (%u..%u ppm): %u ppm",
                 SCD40_CO2_SPEC_MIN,
                 SCD40_CO2_SPEC_MAX,
                 *co2);
    }

    *temperature = s_last_scd40_temp;
    *humidity    = s_last_scd40_hum;

    s_last_scd40_diag = SENSOR_DIAG_OK;
    scd40_set_error("OK");
    return ESP_OK;
}

static esp_err_t sen5x_device_reset(void) {
    uint8_t cmd[2] = {0xD3, 0x04};
    return i2c_master_transmit(s_sen5x_dev, cmd, sizeof(cmd), pdMS_TO_TICKS(1000));
}

static esp_err_t sen5x_start_measurement(void) {
    uint8_t cmd[2] = {0x00, 0x21};
    return i2c_master_transmit(s_sen5x_dev, cmd, sizeof(cmd), pdMS_TO_TICKS(1000));
}

static esp_err_t sen5x_read_data_ready(uint8_t *data_ready) {
    uint8_t cmd[2] = {0x02, 0x02};
    esp_err_t ret = i2c_master_transmit(s_sen5x_dev, cmd, sizeof(cmd), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        s_last_sen55_diag = map_i2c_err_to_diag(ret, false);
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t resp[3];
    ret = i2c_master_receive(s_sen5x_dev, resp, sizeof(resp), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        s_last_sen55_diag = map_i2c_err_to_diag(ret, true);
        return ret;
    }

    if (sensirion_crc8(resp, 2) != resp[2]) {
        s_last_sen55_diag = SENSOR_DIAG_CRC;
        return ESP_ERR_INVALID_CRC;
    }

    *data_ready = resp[1];
    return ESP_OK;
}

static esp_err_t sen5x_read_measured_values(uint8_t *buf, int buflen) {
    uint8_t cmd[2] = {0x03, 0xC4};
    esp_err_t ret = i2c_master_transmit(s_sen5x_dev, cmd, sizeof(cmd), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        s_last_sen55_diag = map_i2c_err_to_diag(ret, false);
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    ret = i2c_master_receive(s_sen5x_dev, buf, buflen, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        s_last_sen55_diag = map_i2c_err_to_diag(ret, true);
        return ret;
    }

    return ESP_OK;
}

static int sen5x_decode_measurement(const uint8_t *buf,
                                    float *pm1,
                                    float *pm25,
                                    float *pm4,
                                    float *pm10,
                                    float *rh,
                                    float *temp,
                                    float *voc_index,
                                    float *nox_index) {
    uint16_t values[8];

    for (int i = 0; i < 8; i++) {
        const uint8_t *data = &buf[i * 3];
        if (sensirion_crc8(data, 2) != data[2]) {
            s_last_sen55_diag = SENSOR_DIAG_CRC;
            return 0;
        }
        values[i] = ((uint16_t)data[0] << 8) | data[1];
    }

    *pm1       = values[0] / 10.0f;
    *pm25      = values[1] / 10.0f;
    *pm4       = values[2] / 10.0f;
    *pm10      = values[3] / 10.0f;
    *rh        = values[4] / 100.0f;
    *temp      = values[5] / 200.0f;
    *voc_index = values[6] / 10.0f;
    *nox_index = values[7] / 10.0f;

    return 1;
}

esp_err_t sensors_init_all(void) {
    ESP_LOGI(TAG_SENS, "Init I2C + sensors...");
    vTaskDelay(pdMS_TO_TICKS(1500));

    if (s_i2c_bus) {
        ESP_LOGD(TAG_SENS, "I2C bus ya inicializado");
        return ESP_OK;
    }

    vTaskDelay(pdMS_TO_TICKS(1500));

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true }
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (ret != ESP_OK) return ret;

    i2c_device_config_t scd_cfg = {
        .device_address = SCD4X_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus, &scd_cfg, &s_scd4x_dev);
    if (ret != ESP_OK) return ret;

    i2c_device_config_t sen_cfg = {
        .device_address = SEN5X_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus, &sen_cfg, &s_sen5x_dev);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(200));
    sen5x_device_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
    sen5x_start_measurement();
    vTaskDelay(pdMS_TO_TICKS(50));
    scd4x_start_measurement();
    vTaskDelay(pdMS_TO_TICKS(5000));

    sensors_reset_diag();
    return ESP_OK;
}

esp_err_t sensors_read_scd40(SensorData *out) {
    if (!out) return ESP_ERR_INVALID_ARG;

    uint16_t co2 = 0;
    float temp = 0.0f;
    float hum = 0.0f;

    esp_err_t ret = scd4x_read_measurement(&co2, &temp, &hum);
    out->co2 = co2;
    out->scd_temp = temp;
    out->scd_hum = hum;

    return ret;
}

esp_err_t sensors_read_sen55(SensorData *out) {
    if (!out) return ESP_ERR_INVALID_ARG;

    s_last_sen55_diag = SENSOR_DIAG_OK;

    uint8_t data_ready = 0;
    bool ready = false;
    esp_err_t ret;

    for (int i = 0; i < SEN55_READY_POLLS; ++i) {
        ret = sen5x_read_data_ready(&data_ready);
        if (ret != ESP_OK) {
            return ret;
        }

        if (data_ready == 1) {
            ready = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(SEN55_READY_DELAY_MS));
    }

    if (!ready) {
        s_last_sen55_diag = SENSOR_DIAG_TIMEOUT;
        return ESP_ERR_TIMEOUT;
    }

    uint8_t buf[24];
    ret = sen5x_read_measured_values(buf, sizeof(buf));
    if (ret != ESP_OK) {
        return ret;
    }

    float pm1, pm25, pm4, pm10, rh, temp, voc, nox;
    if (!sen5x_decode_measurement(buf, &pm1, &pm25, &pm4, &pm10, &rh, &temp, &voc, &nox)) {
        return ESP_ERR_INVALID_CRC;
    }

    out->pm1p0 = pm1;
    out->pm2p5 = pm25;
    out->pm4p0 = pm4;
    out->pm10p0 = pm10;
    out->voc = voc;
    out->nox = nox;
    out->sen_temp = temp;
    out->sen_hum = rh;

    if (out->scd_temp != 0.0f || out->scd_hum != 0.0f) {
        out->avg_temp = (out->scd_temp + temp) / 2.0f;
        out->avg_hum  = (out->scd_hum + rh) / 2.0f;
    } else {
        out->avg_temp = temp;
        out->avg_hum  = rh;
    }

    s_last_sen55_diag = SENSOR_DIAG_OK;
    return ESP_OK;
}

esp_err_t sensors_read(SensorData *out) {
    if (!out) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = sensors_read_scd40(out);
    if (ret != ESP_OK) return ret;

    ret = sensors_read_sen55(out);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

void sensors_format_json(const SensorData *d, char *buf, size_t buf_size) {
    if (!buf || buf_size == 0 || !d) return;

    int written = snprintf(
        buf, buf_size,
        "{\"co2\":%u,\"pm1p0\":%.2f,\"pm2p5\":%.2f,\"pm4p0\":%.2f,\"pm10p0\":%.2f,"
        "\"voc\":%.1f,\"nox\":%.1f,\"temp\":%.2f,\"hum\":%.2f,"
        "\"scd_temp\":%.2f,\"scd_hum\":%.2f,\"sen_temp\":%.2f,\"sen_hum\":%.2f}",
        d->co2,
        d->pm1p0,
        d->pm2p5,
        d->pm4p0,
        d->pm10p0,
        d->voc,
        d->nox,
        d->avg_temp,
        d->avg_hum,
        d->scd_temp,
        d->scd_hum,
        d->sen_temp,
        d->sen_hum
    );

    if (written < 0 || (size_t)written >= buf_size) {
        buf[buf_size - 1] = '\0';
    }
}
