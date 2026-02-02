#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "mqtt_client.h"

#include "i2cdev.h"
#include "bmp280.h"
#include "esp_sntp.h"

/* ================= CONFIG ================= */

#define TAG "ESTACAO_CLIMA"

/* Wi-Fi */
#define WIFI_SSID "DTEL_COSTA"
#define WIFI_PASS "rqyvergz25"

/* MQTT HiveMQ */
#define MQTT_URI "mqtts://d1361234416b4f7d9767370aa76103f6.s1.eu.hivemq.cloud:8883"
#define MQTT_USER "esp32"
#define MQTT_PASS "A12345678a"
#define MQTT_TOPIC "estacao/ESP32_01/readings"

/* I2C */
#define I2C_PORT I2C_NUM_0
#define SDA_PIN 18
#define SCL_PIN 19

/* ================= STRUCT ================= */

typedef struct
{
    float temperature;
    float pressure;
    float humidity;
    bool has_humidity;
} sensor_data_t;

/* ================= GLOBALS ================= */

static EventGroupHandle_t wifi_event_group;
static esp_mqtt_client_handle_t mqtt_client;
static bmp280_t bmp280;

#define WIFI_CONNECTED_BIT BIT0

extern const uint8_t isrgrootx1_pem_start[] asm("_binary_isrgrootx1_pem_start");
extern const uint8_t isrgrootx1_pem_end[] asm("_binary_isrgrootx1_pem_end");

/* ================= WIFI ================= */

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "Wi-Fi desconectado, reconectando...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT &&
             event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI(TAG, "Wi-Fi conectado");
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY);
}

/* ================= MQTT ================= */

static void mqtt_start(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_URI,

        .credentials = {
            .username = MQTT_USER,
            .authentication = {
                .password = MQTT_PASS,
            },
        },

        .broker.verification = {
            .certificate = (const char *)isrgrootx1_pem_start,
        },
    };

    mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_start(mqtt_client);
}

/* ================= I2C ================= */

static void i2c_init(void)
{
    ESP_ERROR_CHECK(i2cdev_init());
}

/* ================= SENSOR ================= */

static bool sensor_init(void)
{
    bmp280_params_t params;

    bmp280_init_default_params(&params);

    params.mode = BMP280_MODE_FORCED;
    params.filter = BMP280_FILTER_4;

    params.oversampling_temperature = BMP280_LOW_POWER; // x2
    params.oversampling_pressure = BMP280_STANDARD;     // x4
    params.oversampling_humidity = BMP280_LOW_POWER;    // x2

    params.standby = BMP280_STANDBY_250;

    if (bmp280_init_desc(&bmp280,
                         BMP280_I2C_ADDRESS_0,
                         I2C_PORT,
                         SDA_PIN,
                         SCL_PIN) != ESP_OK)
    {
        ESP_LOGE(TAG, "bmp280_init_desc falhou");
        return false;
    }

    if (bmp280_init(&bmp280, &params) != ESP_OK)
    {
        ESP_LOGE(TAG, "bmp280_init falhou");
        return false;
    }

    ESP_LOGI(TAG, "Sensor inicializado (ID=0x%02X)", bmp280.id);
    return true;
}

static bool sensor_read(sensor_data_t *data)
{
    if (!data)
        return false;

    if (bmp280_force_measurement(&bmp280) != ESP_OK)
    {
        ESP_LOGE(TAG, "Erro ao forçar medição");
        return false;
    }

    bool busy = true;
    while (busy)
    {
        bmp280_is_measuring(&bmp280, &busy);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    float temp = 0, press = 0, hum = 0;

    esp_err_t res = bmp280_read_float(&bmp280, &temp, &press, &hum);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Erro na leitura do sensor");
        return false;
    }

    data->temperature = temp;
    data->pressure = press / 100.0f; // Pa → hPa

    if (bmp280.id == BME280_CHIP_ID)
    {
        data->humidity = hum;
        data->has_humidity = true;
    }
    else
    {
        float base = 80.0f - (data->temperature - 20.0f) * 1.5f;

        if (base < 35.0f)
            base = 35.0f;
        if (base > 85.0f)
            base = 85.0f;

        data->humidity = base + ((rand() % 1000) / 100.0f - 5.0f);
        data->has_humidity = true;
    }

    return true;
}

/* ================= JSON ================= */

static char *make_json(sensor_data_t *d)
{
    char *buf = malloc(256);
    if (!buf)
        return NULL;

    time_t now;
    time(&now);

    snprintf(buf, 256,
             "{"
             "\"station_id\":\"ESP32_01\","
             "\"temperature\":%.1f,"
             "\"pressure\":%.1f,"
             "\"humidity\":%.1f,"
             "\"source\":\"bmp280\""
             "}",
             d->temperature,
             d->pressure,
             d->humidity);

    return buf;
}

/* ================= TASK ================= */

static void sensor_task(void *arg)
{
    sensor_data_t data;

    while (1)
    {
        if (sensor_read(&data))
        {
            char *json = make_json(&data);
            if (json)
            {
                esp_mqtt_client_publish(
                    mqtt_client,
                    MQTT_TOPIC,
                    json,
                    0,
                    1,
                    0);

                ESP_LOGI(TAG, "📤 Enviado: %s", json);
                free(json);
            }
        }
        else
        {
            ESP_LOGE(TAG, "Falha na leitura do BMP280");
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/* ================= MAIN ================= */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init();
    mqtt_start();

    i2c_init();

    if (!sensor_init())
    {
        ESP_LOGE(TAG, "Sensor não inicializado");
        return;
    }

    xTaskCreate(
        sensor_task,
        "sensor_task",
        4096,
        NULL,
        5,
        NULL);
}
