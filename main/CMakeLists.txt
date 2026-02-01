idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_netif
        esp_wifi
        nvs_flash
        bmp280
        i2cdev
        mqtt
    EMBED_TXTFILES
        certs/isrgrootx1.pem
)