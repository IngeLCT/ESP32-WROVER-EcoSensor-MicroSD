#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cJSON.h"
#include "esp_err.h"
#include "captive_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SD_STORE_PIN_SCK   14
#define SD_STORE_PIN_MISO  2
#define SD_STORE_PIN_MOSI  15
#define SD_STORE_PIN_CS    13

esp_err_t sd_store_init(void);
bool sd_store_is_ready(void);
uint32_t sd_store_last_id(void);
esp_err_t sd_store_append_reading(const captive_manager_readings_t *reading, uint32_t *out_id);
esp_err_t sd_store_add_readings_since(cJSON *array, uint32_t after_id, uint32_t limit, uint32_t timeout_ms, uint32_t *added, uint32_t *scanned);
esp_err_t sd_store_add_recent_readings(cJSON *array, uint32_t after_id, uint32_t before_id, uint32_t limit, uint32_t timeout_ms, uint32_t *added, uint32_t *scanned);
esp_err_t sd_store_clear(void);

#ifdef __cplusplus
}
#endif
