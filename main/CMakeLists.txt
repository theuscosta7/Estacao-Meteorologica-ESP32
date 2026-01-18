idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_http_client
        esp_netif
        nvs_flash
        bme680
        driver  # <-- ADICIONE ESTA LINHA
)