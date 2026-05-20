idf_component_register(
    SRCS "src/captive_manager.c" "src/wifi_store.c" "src/sd_store.c" "src/ota_manager.c"
    INCLUDE_DIRS "." "include"
    REQUIRES esp_wifi esp_event esp_http_server esp_http_client app_update esp_app_format esp_netif nvs_flash esp_timer json mdns fatfs sdmmc driver
)
