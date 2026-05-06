#pragma once
#include "esp_err.h"
#include "stdbool.h"

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

esp_err_t captive_manager_init(const captive_manager_cfg_t *cfg);
esp_err_t captive_manager_start(void);

captive_state_t captive_manager_get_state(void);
const char* captive_manager_state_str(captive_state_t st);

void captive_manager_notify_sta_got_ip(void);
void captive_manager_notify_sta_disconnected(int reason_code);

esp_err_t captive_manager_enter_recaptive(void);
bool captive_manager_using_saved(void);

#ifdef __cplusplus
}
#endif
