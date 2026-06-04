#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    double lat;
    double lon;
    uint8_t satellites;
    float hdop;
    uint32_t age_ms;
} gps_fix_t;

esp_err_t gps_init(void);
bool gps_has_valid_fix(void);
esp_err_t gps_wait_for_valid(uint32_t timeout_ms);
esp_err_t gps_get_fix(gps_fix_t *out, uint32_t max_age_ms, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
