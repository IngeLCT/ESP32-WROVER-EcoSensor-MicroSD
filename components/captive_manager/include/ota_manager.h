#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cJSON.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OTA_MANAGER_URL_MAX_LEN 256
#define OTA_MANAGER_VERSION_MAX_LEN 32
#define OTA_MANAGER_SHA256_MAX_LEN 65
#define OTA_MANAGER_ERROR_MAX_LEN 160

typedef enum {
    OTA_MANAGER_STATE_IDLE = 0,
    OTA_MANAGER_STATE_QUEUED,
    OTA_MANAGER_STATE_DOWNLOADING,
    OTA_MANAGER_STATE_WRITING,
    OTA_MANAGER_STATE_SUCCESS,
    OTA_MANAGER_STATE_ERROR,
    OTA_MANAGER_STATE_REBOOTING,
} ota_manager_state_t;

esp_err_t ota_manager_init(void);
const char *ota_manager_current_version(void);
const char *ota_manager_state_name(ota_manager_state_t state);
esp_err_t ota_manager_start(const char *device_id, const char *target_version, const char *firmware_url, const char *sha256);
void ota_manager_add_status_json(cJSON *root);
bool ota_manager_busy(void);

#ifdef __cplusplus
}
#endif
