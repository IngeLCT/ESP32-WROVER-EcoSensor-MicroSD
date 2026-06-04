#include "gps.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define GPS_UART_PORT          UART_NUM_2
#define GPS_TX_PIN             GPIO_NUM_21
#define GPS_RX_PIN             GPIO_NUM_22
#define GPS_WAKEUP_PIN         GPIO_NUM_19
#define GPS_BAUD_RATE          9600
#define GPS_TASK_STACK         4096
#define GPS_UART_BUF_SIZE      1024
#define GPS_LINE_MAX           128

static const char *TAG = "GPS";

static TaskHandle_t s_gps_task_handle = NULL;
static SemaphoreHandle_t s_gps_lock = NULL;
static gps_fix_t s_last_fix = {0};
static TickType_t s_last_fix_tick = 0;
static bool s_started = false;

static bool starts_with(const char *text, const char *prefix) {
    return text && prefix && strncmp(text, prefix, strlen(prefix)) == 0;
}

static bool checksum_ok(const char *line) {
    if (!line || line[0] != '$') {
        return false;
    }
    const char *star = strchr(line, '*');
    if (!star || strlen(star) < 3) {
        return true;  // Algunos módulos pueden enviar sentencias sin checksum; no las rechazamos.
    }
    uint8_t sum = 0;
    for (const char *p = line + 1; p < star; ++p) {
        sum ^= (uint8_t)(*p);
    }
    char expected[3] = { star[1], star[2], 0 };
    char *end = NULL;
    unsigned long received = strtoul(expected, &end, 16);
    return end && *end == '\0' && sum == (uint8_t)received;
}

static int split_nmea(char *line, char **fields, int max_fields) {
    int count = 0;
    char *start = line;
    if (*start == '$') {
        start++;
    }
    for (char *p = start; *p && count < max_fields; ++p) {
        if (*p == '*' || *p == '\r' || *p == '\n') {
            *p = '\0';
            break;
        }
        if (*p == ',') {
            *p = '\0';
            fields[count++] = start;
            start = p + 1;
        }
    }
    if (count < max_fields) {
        fields[count++] = start;
    }
    return count;
}

static bool parse_coordinate(const char *value, const char *hemisphere, double *out) {
    if (!value || !hemisphere || !out || !value[0] || !hemisphere[0]) {
        return false;
    }
    char *end = NULL;
    double raw = strtod(value, &end);
    if (!end || end == value || raw <= 0.0) {
        return false;
    }
    int degrees = (int)(raw / 100.0);
    double minutes = raw - ((double)degrees * 100.0);
    double decimal = (double)degrees + (minutes / 60.0);
    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
        decimal = -decimal;
    } else if (hemisphere[0] != 'N' && hemisphere[0] != 'E') {
        return false;
    }
    *out = decimal;
    return fabs(decimal) <= 180.0;
}

static void store_fix(double lat, double lon, uint8_t satellites, float hdop) {
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
        return;
    }
    if (s_gps_lock) {
        xSemaphoreTake(s_gps_lock, portMAX_DELAY);
    }
    uint8_t previous_satellites = s_last_fix.satellites;
    float previous_hdop = s_last_fix.hdop;
    s_last_fix.valid = true;
    s_last_fix.lat = lat;
    s_last_fix.lon = lon;
    s_last_fix.satellites = satellites ? satellites : previous_satellites;
    s_last_fix.hdop = hdop > 0.0f ? hdop : previous_hdop;
    s_last_fix.age_ms = 0;
    s_last_fix_tick = xTaskGetTickCount();
    if (s_gps_lock) {
        xSemaphoreGive(s_gps_lock);
    }
}

static void parse_rmc(char **fields, int n) {
    // RMC: type,time,status,lat,N,lon,E,speed,course,date,...
    if (n < 7 || !fields[2] || fields[2][0] != 'A') {
        return;
    }
    double lat = 0.0;
    double lon = 0.0;
    if (parse_coordinate(fields[3], fields[4], &lat) && parse_coordinate(fields[5], fields[6], &lon)) {
        store_fix(lat, lon, 0, 0.0f);
    }
}

static void parse_gga(char **fields, int n) {
    // GGA: type,time,lat,N,lon,E,quality,sats,hdop,...
    if (n < 9) {
        return;
    }
    int quality = atoi(fields[6] ? fields[6] : "0");
    if (quality <= 0) {
        return;
    }
    double lat = 0.0;
    double lon = 0.0;
    if (!parse_coordinate(fields[2], fields[3], &lat) || !parse_coordinate(fields[4], fields[5], &lon)) {
        return;
    }
    int sats = atoi(fields[7] ? fields[7] : "0");
    float hdop = fields[8] ? strtof(fields[8], NULL) : 0.0f;
    store_fix(lat, lon, (uint8_t)(sats < 0 ? 0 : sats > 255 ? 255 : sats), hdop);
}

static void parse_line(const char *line) {
    if (!checksum_ok(line)) {
        return;
    }
    char copy[GPS_LINE_MAX];
    snprintf(copy, sizeof(copy), "%s", line);
    char *fields[24] = {0};
    int n = split_nmea(copy, fields, 24);
    if (n <= 0 || !fields[0]) {
        return;
    }
    if (starts_with(fields[0], "GPRMC") || starts_with(fields[0], "GNRMC")) {
        parse_rmc(fields, n);
    } else if (starts_with(fields[0], "GPGGA") || starts_with(fields[0], "GNGGA")) {
        parse_gga(fields, n);
    }
}

static void gps_task(void *pv) {
    (void)pv;
    uint8_t byte = 0;
    char line[GPS_LINE_MAX];
    size_t pos = 0;
    uint32_t chars_seen = 0;
    TickType_t last_diag = xTaskGetTickCount();
    bool first_fix_logged = false;

    ESP_LOGI(TAG, "Tarea GPS iniciada en UART%d RX=%d TX=%d baud=%d", GPS_UART_PORT, GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE);

    while (1) {
        int len = uart_read_bytes(GPS_UART_PORT, &byte, 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            chars_seen++;
            if (byte == '\n') {
                line[pos] = '\0';
                if (pos > 0) {
                    parse_line(line);
                }
                pos = 0;
            } else if (byte != '\r') {
                if (pos + 1 < sizeof(line)) {
                    line[pos++] = (char)byte;
                } else {
                    pos = 0;
                }
            }
        }

        gps_fix_t fix = {0};
        if (gps_get_fix(&fix, 600000, 0) == ESP_OK && !first_fix_logged) {
            ESP_LOGI(TAG, "Primer fix GPS valido: lat=%.6f lon=%.6f sats=%u hdop=%.2f", fix.lat, fix.lon, fix.satellites, fix.hdop);
            first_fix_logged = true;
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_diag) >= pdMS_TO_TICKS(30000)) {
            if (!first_fix_logged) {
                ESP_LOGW(TAG, "Esperando fix GPS valido; caracteres NMEA recibidos=%lu", (unsigned long)chars_seen);
            }
            last_diag = now;
        }
    }
}

esp_err_t gps_init(void) {
    if (s_started) {
        return ESP_OK;
    }
    if (!s_gps_lock) {
        s_gps_lock = xSemaphoreCreateMutex();
        if (!s_gps_lock) {
            return ESP_ERR_NO_MEM;
        }
    }

    gpio_config_t wake_cfg = {
        .pin_bit_mask = 1ULL << GPS_WAKEUP_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&wake_cfg);
    if (err == ESP_OK) {
        gpio_set_level(GPS_WAKEUP_PIN, 1);
    } else {
        ESP_LOGW(TAG, "No se pudo configurar GPS_WAKEUP GPIO%d: %s", GPS_WAKEUP_PIN, esp_err_to_name(err));
    }

    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(GPS_UART_PORT, GPS_UART_BUF_SIZE, 0, 0, NULL, 0), TAG, "uart_driver_install GPS");
    ESP_RETURN_ON_ERROR(uart_param_config(GPS_UART_PORT, &uart_config), TAG, "uart_param_config GPS");
    ESP_RETURN_ON_ERROR(uart_set_pin(GPS_UART_PORT, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "uart_set_pin GPS");

    BaseType_t ok = xTaskCreate(gps_task, "gps_task", GPS_TASK_STACK, NULL, tskIDLE_PRIORITY + 2, &s_gps_task_handle);
    if (ok != pdPASS) {
        uart_driver_delete(GPS_UART_PORT);
        return ESP_ERR_NO_MEM;
    }
    s_started = true;
    ESP_LOGI(TAG, "GPS inicializado; se esperara primer fix valido antes de medir");
    return ESP_OK;
}

bool gps_has_valid_fix(void) {
    bool valid = false;
    if (s_gps_lock) {
        xSemaphoreTake(s_gps_lock, portMAX_DELAY);
    }
    valid = s_last_fix.valid;
    if (s_gps_lock) {
        xSemaphoreGive(s_gps_lock);
    }
    return valid;
}

esp_err_t gps_get_fix(gps_fix_t *out, uint32_t max_age_ms, uint32_t timeout_ms) {
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    TickType_t started = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    TickType_t max_age_ticks = pdMS_TO_TICKS(max_age_ms);

    while (1) {
        if (s_gps_lock) {
            xSemaphoreTake(s_gps_lock, portMAX_DELAY);
        }
        bool valid = s_last_fix.valid;
        TickType_t now = xTaskGetTickCount();
        TickType_t age_ticks = now - s_last_fix_tick;
        bool fresh = valid && (max_age_ms == 0 || age_ticks <= max_age_ticks);
        if (fresh) {
            *out = s_last_fix;
            out->age_ms = (uint32_t)(age_ticks * portTICK_PERIOD_MS);
        }
        if (s_gps_lock) {
            xSemaphoreGive(s_gps_lock);
        }
        if (fresh) {
            return ESP_OK;
        }
        if (timeout_ms == 0 || (xTaskGetTickCount() - started) >= timeout_ticks) {
            return valid ? ESP_ERR_TIMEOUT : ESP_ERR_NOT_FOUND;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

esp_err_t gps_wait_for_valid(uint32_t timeout_ms) {
    gps_fix_t fix = {0};
    return gps_get_fix(&fix, 0, timeout_ms);
}
