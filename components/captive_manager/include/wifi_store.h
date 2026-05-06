#pragma once
#include "stdbool.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool enabled;
    char ip[16];
    char gateway[16];
} wifi_static_ip_cfg_t;

bool wifi_store_has_credentials(void);
esp_err_t wifi_store_load(char *ssid, size_t ssid_len, char *pass, size_t pass_len);
esp_err_t wifi_store_save(const char *ssid, const char *pass);
esp_err_t wifi_store_clear(void);

esp_err_t wifi_store_save_static_ip_cfg(const wifi_static_ip_cfg_t *cfg);
esp_err_t wifi_store_load_static_ip_cfg(wifi_static_ip_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
