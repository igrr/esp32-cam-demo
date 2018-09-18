#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "xclk.h"

static const char* TAG = "camera_xclk";

esp_err_t camera_enable_out_clock(camera_config_t* config)
{
    periph_module_enable(PERIPH_LEDC_MODULE);

    ledc_timer_config_t timer_conf = {
        .duty_resolution = 1,
        .freq_hz = config->xclk_freq_hz,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = config->ledc_timer
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
        return err;
    }

    ledc_channel_config_t ch_conf = {
        .channel = config->ledc_channel,
        .timer_sel = config->ledc_timer,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 1,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .gpio_num = config->pin_xclk
    };
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
        return err;
    }
    return ESP_OK;
}

void camera_disable_out_clock()
{
    periph_module_disable(PERIPH_LEDC_MODULE);
}
