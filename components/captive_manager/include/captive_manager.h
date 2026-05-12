#pragma once
#include "esp_err.h"
#include "stdbool.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAP_STATE_IDLE = 0,
    CAP_STATE_PREP,
    CAP_STATE_SCAN,
    CAP_STATE_CONNECTING,
    CAP_STATE_WAIT_LOGIN,
    CAP_STATE_VERIFY,
    CAP_STATE_OPERATIONAL,
    CAP_STATE_RECAPTIVE
} captive_state_t;

typedef struct {
    const char *ap_ssid;
    const char *ap_pass;
    int max_scan_aps;
    int conn_max_attempts;
    int conn_retry_delay_ms;
    int boot_grace_ms;
    const char *mdns_hostname;
} captive_manager_cfg_t;

typedef struct {
    bool valid;
    uint32_t id;
    uint32_t measurement_id;
    uint32_t boot_id;
    uint32_t uptime_s;
    uint32_t window_s;
    bool time_valid;
    char timestamp[32];
    uint16_t co2;
    float pm1p0;
    float pm2p5;
    float pm4p0;
    float pm10p0;
    float voc;
    float nox;
    float temp;
    float hum;
} captive_manager_readings_t;

esp_err_t captive_manager_init(const captive_manager_cfg_t *cfg);
esp_err_t captive_manager_start(void);

captive_state_t captive_manager_get_state(void);
const char* captive_manager_state_str(captive_state_t st);

void captive_manager_notify_sta_got_ip(void);
void captive_manager_notify_sta_disconnected(int reason_code);

esp_err_t captive_manager_enter_recaptive(void);
bool captive_manager_using_saved(void);
void captive_manager_set_sensors_started(bool started);
void captive_manager_set_last_readings(const captive_manager_readings_t *readings);
bool captive_manager_time_is_valid(void);
uint32_t captive_manager_boot_id(void);

#ifdef __cplusplus
}
#endif
