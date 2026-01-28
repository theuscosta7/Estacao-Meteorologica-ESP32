#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "protocol_examples_common.h"
#include "esp_http_client.h"

#include "driver/i2c.h"

static const char *TAG = "ESTACAO_CLIMA";

/* ================= CONFIGURAÇÕES ================= */
#define I2C_SDA_GPIO 18
#define I2C_SCL_GPIO 19
#define I2C_PORT I2C_NUM_1
#define BME680_ADDR 0x76

#define WEB_URL "http://192.168.18.159:3000/api/createReadings"

/* ================= ESTRUTURAS ================= */
typedef struct
{
    float temperature;
    float humidity;
    float pressure;
    float gas_resistance;
    float co2;
    bool is_real_data;
} sensor_data_t;

/* ================= VARIÁVEIS GLOBAIS ================= */
static bool sensor_available = false;

/* ================= FUNÇÕES I2C ================= */
static esp_err_t i2c_write(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME680_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME680_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME680_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ================= INICIALIZAÇÃO ================= */
static void init_i2c(void)
{
    ESP_LOGI(TAG, "Inicializando I2C...");

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));

    ESP_LOGI(TAG, "✓ I2C inicializado");
}

/* ================= VERIFICAÇÃO DO SENSOR ================= */
static bool check_sensor(void)
{
    ESP_LOGI(TAG, "Verificando sensor BME680...");

    uint8_t chip_id;
    if (i2c_read(0xD0, &chip_id, 1) != ESP_OK)
    {
        ESP_LOGE(TAG, "Não conseguiu ler Chip ID");
        return false;
    }

    ESP_LOGI(TAG, "Chip ID: 0x%02X", chip_id);

    if (chip_id != 0x60 && chip_id != 0x61)
    {
        ESP_LOGW(TAG, "Chip ID não corresponde ao BME680");
        return false;
    }

    // Tenta configurar
    i2c_write(0xE0, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(100));

    i2c_write(0x72, 0x02); 
    i2c_write(0x74, 0x2B); 
    i2c_write(0x75, 0x14); 

    ESP_LOGI(TAG, "BME680 detectado (mesmo que possa estar defeituoso)");
    return true;
}

/* ================= TENTA LER SENSOR REAL ================= */
static bool try_read_real_sensor(sensor_data_t *data)
{
    if (!sensor_available)
        return false;

    
    i2c_write(0x74, 0x2B);
    vTaskDelay(pdMS_TO_TICKS(200));

    
    uint8_t raw[15];
    if (i2c_read(0x1F, raw, 15) != ESP_OK)
    {
        return false;
    }


    bool all_zeros = true;
    for (int i = 0; i < 15; i++)
    {
        if (raw[i] != 0)
        {
            all_zeros = false;
            break;
        }
    }

    if (all_zeros)
    {
        ESP_LOGW(TAG, "Dados crus todos zero - sensor defeituoso");
        return false;
    }

    
    uint32_t temp_raw = ((uint32_t)raw[5] << 12) | ((uint32_t)raw[6] << 4) | (raw[7] >> 4);
    uint32_t press_raw = ((uint32_t)raw[2] << 12) | ((uint32_t)raw[3] << 4) | (raw[4] >> 4);
    uint16_t hum_raw = ((uint16_t)raw[7] << 8) | raw[8];
    uint16_t gas_raw = ((uint16_t)raw[13] << 2) | (raw[14] >> 6);

    data->temperature = temp_raw / 16384.0;
    data->pressure = press_raw / 16.0 / 100.0;
    data->humidity = hum_raw / 512.0;
    data->gas_resistance = gas_raw * 10.0;

    if (data->gas_resistance > 0)
    {
        data->co2 = 400.0 + (10000.0 / data->gas_resistance) * 100.0;
    }
    else
    {
        data->co2 = 400.0;
    }

    if (data->temperature < -40 || data->temperature > 85 ||
        data->pressure < 300 || data->pressure > 1100)
    {
        return false;
    }

    data->is_real_data = true;
    return true;
}

/* ================= GERA DADOS SIMULADOS ================= */
static void generate_simulated_data(sensor_data_t *data, int cycle)
{
    
    float base_temp = 25.0;
    float base_hum = 55.0;
    float base_press = 1013.0;

    float temp_variation = sin(cycle * 0.1) * 3.0;
    float hum_variation = sin(cycle * 0.15) * 10.0;
    float press_variation = sin(cycle * 0.05) * 5.0;

    data->temperature = base_temp + temp_variation;
    data->humidity = base_hum + hum_variation;
    data->pressure = base_press + press_variation;
    data->gas_resistance = 12000 + sin(cycle * 0.2) * 5000;
    data->co2 = 500 + sin(cycle * 0.25) * 200;
    data->is_real_data = false;

    if (data->humidity < 30)
        data->humidity = 30;
    if (data->humidity > 80)
        data->humidity = 80;
    if (data->co2 < 400)
        data->co2 = 400;
    if (data->co2 > 1500)
        data->co2 = 1500;
}

/* ================= JSON ================= */
static char *create_json_payload(sensor_data_t *data)
{
    char *json = malloc(256);
    if (!json)
        return NULL;

    float co2 = data->co2;
    if (isinf(co2) || isnan(co2))
        co2 = 400.0;

    snprintf(json, 256,
             "{\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f,"
             "\"gas_resistance\":%.0f,\"co2\":%.0f,\"source\":\"%s\"}",
             data->temperature, data->humidity, data->pressure,
             data->gas_resistance, co2,
             data->is_real_data ? "real" : "simulated");

    return json;
}

/* ================= HTTP POST ================= */
static void send_data_to_server(sensor_data_t *data)
{
    char *json_payload = create_json_payload(data);
    if (!json_payload)
    {
        ESP_LOGE(TAG, "Erro ao criar JSON");
        return;
    }

    ESP_LOGI(TAG, "Enviando: %s", json_payload);

    esp_http_client_config_t config = {
        .url = WEB_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_payload, strlen(json_payload));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "POST Status: %d", status);
    }
    else
    {
        ESP_LOGE(TAG, "POST Erro: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    free(json_payload);
}

/* ================= TASK PRINCIPAL ================= */
static void sensor_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Task sensor iniciada");

    sensor_available = check_sensor();

    if (!sensor_available)
    {
        ESP_LOGW(TAG, "Sensor não disponível. Usando modo SIMULADO.");
    }
    else
    {
        ESP_LOGI(TAG, "Sensor disponível. Tentando leituras reais.");
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    int cycle = 0;
    int real_read_fails = 0;
    const int MAX_REAL_FAILS = 2;

    while (1)
    {
        cycle++;
        sensor_data_t data;
        bool real_data_ok = false;

        // Tenta ler dados reais
        if (sensor_available && real_read_fails < MAX_REAL_FAILS)
        {
            real_data_ok = try_read_real_sensor(&data);

            if (real_data_ok)
            {
                ESP_LOGI(TAG, "Dados REAIS: T=%.2f°C H=%.2f%% P=%.2fhPa",
                         data.temperature, data.humidity, data.pressure);
                real_read_fails = 0;
            }
            else
            {
                real_read_fails++;
                ESP_LOGW(TAG, "Falha leitura real (%d/%d)",
                         real_read_fails, MAX_REAL_FAILS);

                if (real_read_fails >= MAX_REAL_FAILS)
                {
                    ESP_LOGE(TAG, "Muitas falhas. Mudando para modo SIMULADO permanente.");
                    sensor_available = false;
                }
            }
        }

        if (!real_data_ok)
        {
            generate_simulated_data(&data, cycle);
            ESP_LOGI(TAG, "Dados SIMULADOS: T=%.2f°C H=%.2f%% P=%.2fhPa",
                     data.temperature, data.humidity, data.pressure);
        }

        send_data_to_server(&data);

        ESP_LOGI(TAG, "Aguardando 30 segundos...");
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

/* ================= MAIN ================= */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   ESTAÇÃO CLIMÁTICA ESP32");
    ESP_LOGI(TAG, "   (Sensor defeituoso)");
    ESP_LOGI(TAG, "========================================");

    // Inicializa NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "Conectando WiFi...");
    ESP_ERROR_CHECK(example_connect());
    ESP_LOGI(TAG, "WiFi conectado!");

    init_i2c();

    vTaskDelay(pdMS_TO_TICKS(2000));

    xTaskCreate(sensor_task, "sensor_task", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "Sistema iniciado com sucesso!");
    ESP_LOGI(TAG, "URL: %s", WEB_URL);
    ESP_LOGI(TAG, "========================================");
}