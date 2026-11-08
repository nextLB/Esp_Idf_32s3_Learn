idf_component_register(
    SRCS "main.c" "wifi_manager.c" "udp_listener.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_http_client lwip
)