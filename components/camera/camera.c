/*
 * Portions of this file come from OpenMV project (see sensor_* functions in the end of file)
 * Here is the copyright for these parts:
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 *
 * Rest of the functions are licensed under Apache license as found below:
 */

// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "soc/soc.h"
#include "sccb.h"
#include "wiring.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "sensor.h"
#if CONFIG_OV2640_SUPPORT
    #include "ov2640.h"
#endif
#if CONFIG_OV7725_SUPPORT
    #include "ov7725.h"
#endif
#include <stdlib.h>
#include <string.h>
#include "rom/lldesc.h"
#include "esp_intr_alloc.h"
#include "camera.h"
#include "esp_log.h"
#include "driver/periph_ctrl.h"

# define ENABLE_TEST_PATTERN CONFIG_ENABLE_TEST_PATTERN

static const char* TAG = "camera";

static camera_config_t s_config;
static lldesc_t s_dma_desc[2];
static uint32_t* s_dma_buf[2];
static uint8_t* s_fb;
static sensor_t s_sensor;
static bool s_initialized = false;
static int s_fb_w;
static int s_fb_h;
static size_t s_fb_size;
static volatile int s_isr_count = 0;
static volatile int s_line_count = 0;
static volatile int s_cur_buffer = 0;
static int s_buf_line_width;
static int s_buf_height;
static volatile bool s_i2s_running = 0;
static SemaphoreHandle_t s_data_ready;
static SemaphoreHandle_t s_frame_ready;
static intr_handle_t s_i2s_intr_handle = NULL;

const int resolution[][2] = {
    {40,    30 },    /* 40x30 */
    {64,    32 },    /* 64x32 */
    {64,    64 },    /* 64x64 */
    {88,    72 },    /* QQCIF */
    {160,   120},    /* QQVGA */
    {128,   160},    /* QQVGA2*/
    {176,   144},    /* QCIF  */
    {240,   160},    /* HQVGA */
    {320,   240},    /* QVGA  */
    {352,   288},    /* CIF   */
    {640,   480},    /* VGA   */
    {800,   600},    /* SVGA  */
    {1280,  1024},   /* SXGA  */
    {1600,  1200},   /* UXGA  */
};

static void i2s_init();
static void i2s_run(size_t line_width, int height);
static void IRAM_ATTR i2s_isr(void* arg);
static esp_err_t dma_desc_init(int line_width);
static void line_filter_task(void *pvParameters);

static void enable_out_clock()
{
    periph_module_enable(PERIPH_LEDC_MODULE);

    ledc_timer_config_t timer_conf;
    timer_conf.bit_num = 3;
    timer_conf.freq_hz = s_config.xclk_freq_hz;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = s_config.ledc_timer;
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
    }

    ledc_channel_config_t ch_conf;
    ch_conf.channel = s_config.ledc_channel;
    ch_conf.timer_sel = s_config.ledc_timer;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.duty = 4;
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.gpio_num = s_config.pin_xclk;
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
    }
}

esp_err_t camera_init(const camera_config_t* config)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memcpy(&s_config, config, sizeof(s_config));
    ESP_LOGD(TAG, "Enabling XCLK output");
    enable_out_clock();
    ESP_LOGD(TAG, "Initializing SSCB");
    SCCB_Init(s_config.pin_sscb_sda, s_config.pin_sscb_scl);
    ESP_LOGD(TAG, "Resetting camera");
    gpio_set_level(s_config.pin_reset, 0);
    delay(1);
    gpio_set_level(s_config.pin_reset, 1);
    delay(10);
    ESP_LOGD(TAG, "Searching for camera address");
    
    /* Probe the sensor */
    s_sensor.slv_addr = SCCB_Probe();
    if (s_sensor.slv_addr == 0) {
        /* Sensor has been held in reset,
           so the reset line is active high */
        s_sensor.reset_pol = ACTIVE_HIGH;

        /* Pull the sensor out of the reset state */
        gpio_set_level(s_config.pin_reset, 0);
        delay(10);

        /* Probe again to set the slave addr */
        s_sensor.slv_addr = SCCB_Probe();
        if (s_sensor.slv_addr == 0)  {
            // Probe failed
            return ESP_ERR_CAMERA_NOT_DETECTED;
        }
    }

    ESP_LOGD(TAG, "Detected camera at address=0x%02x", s_sensor.slv_addr);
    
    s_sensor.id.PID  = SCCB_Read(s_sensor.slv_addr, REG_PID);
    s_sensor.id.VER  = SCCB_Read(s_sensor.slv_addr, REG_VER);
    s_sensor.id.MIDL = SCCB_Read(s_sensor.slv_addr, REG_MIDL);
    s_sensor.id.MIDH = SCCB_Read(s_sensor.slv_addr, REG_MIDH);

    ESP_LOGD(TAG, "Camera PID=0x%02x VER=0x%02x MIDL=0x%02x MIDH=0x%02x",
        s_sensor.id.pid, s_sensor.id.ver, s_sensor.id.midh,
        s_sensor.id.midl);

    switch (s_sensor.id.PID) {
#if CONFIG_OV2640_SUPPORT
        case OV2640_PID:
            ov2640_init(&s_sensor);
            break;
#endif
#if CONFIG_OV7725_SUPPORT
        case OV7725_PID:
            ov7725_init(&s_sensor);
            break;
#endif
        default:
            /* Sensor not supported */
            ESP_LOGD(TAG, "Detected camera not supported.");
            return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }

    ESP_LOGD(TAG, "Doing SW reset of sensor");
    s_sensor.reset(&s_sensor);

#if ENABLE_TEST_PATTERN
    /* Test pattern may get handy
       if you are unable to get the live image right.
       Once test pattern is enable, sensor will output
       vertical shaded bars instead of live image.
    */
    s_sensor.set_colorbar(&s_sensor, 1);
    ESP_LOGD(TAG, "Test pattern enabled");
#endif

    framesize_t framesize = FRAMESIZE_QVGA;
    s_fb_w = resolution[framesize][0];
    s_fb_h = resolution[framesize][1];
    ESP_LOGD(TAG, "Setting frame size at %dx%d", s_fb_w, s_fb_h);
    if (s_sensor.set_framesize(&s_sensor, framesize) != 0) {
        ESP_LOGE(TAG, "Failed to set frame size");
        return ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE;
    }

    s_fb_size = s_fb_w * s_fb_h;
    ESP_LOGD(TAG, "Allocating frame buffer (%dx%d, %d bytes)", s_fb_w, s_fb_h,
            s_fb_size);
    s_fb = (uint8_t*) malloc(s_fb_size);
    if (s_fb == NULL) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGD(TAG, "Initializing I2S and DMA");
    i2s_init();
    esp_err_t err = dma_desc_init(s_fb_w);
    if (err != ESP_OK) {
        free(s_fb);
        return err;
    }

    s_data_ready = xSemaphoreCreateBinary();
    s_frame_ready = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(&line_filter_task, "line_filter", 2048, NULL, 10,
            NULL, 0);

    // skip at least one frame after changing camera settings
    while (gpio_get_level(s_config.pin_vsync) == 0) {
        ;
    }
    while (gpio_get_level(s_config.pin_vsync) != 0) {
        ;
    }
    while (gpio_get_level(s_config.pin_vsync) == 0) {
        ;
    }

    ESP_LOGD(TAG, "Init done");
    s_initialized = true;
    return ESP_OK;
}

uint8_t* camera_get_fb()
{
    if (!s_initialized) {
        return NULL;
    }
    return s_fb;
}

int camera_get_fb_width()
{
    if (!s_initialized) {
        return 0;
    }
    return s_fb_w;
}

int camera_get_fb_height()
{
    if (!s_initialized) {
        return 0;
    }
    return s_fb_h;
}

esp_err_t camera_run()
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    i2s_run(s_fb_w, s_fb_h);
    ESP_LOGD(TAG, "Waiting for frame");
    xSemaphoreTake(s_frame_ready, portMAX_DELAY);
    ESP_LOGD(TAG, "Frame done");
    return ESP_OK;
}

void camera_print_fb()
{
	/* Number of pixels to skip
	   in order to fit into terminal screen.
	   Assumed picture to be 80 columns wide.
	   Skip twice as more rows as they look higher.
	 */
	int pixels_to_skip = s_fb_w / 80;

    for (int ih = 0; ih < s_fb_h; ih += pixels_to_skip * 2){
        for (int iw = 0; iw < s_fb_w; iw += pixels_to_skip){
    	    uint8_t px = (s_fb[iw + (ih * s_fb_w)]);
    	    if      (px <  26) printf(" ");
    	    else if (px <  51) printf(".");
    	    else if (px <  77) printf(":");
    	    else if (px < 102) printf("-");
    	    else if (px < 128) printf("=");
    	    else if (px < 154) printf("+");
    	    else if (px < 179) printf("*");
    	    else if (px < 205) printf("#");
    	    else if (px < 230) printf("%%");
    	    else               printf("@");
        }
        printf("\n");
    }
}

static esp_err_t dma_desc_init(int line_width)
{
	/* I2S peripheral captures 16 bit of data every clock cycle,
	   even though we are only using 8 bits.
	   On top of that we need two bytes per pixel.
	 */
    size_t buf_size = line_width * 4;
    for (int i = 0; i < 2; ++i) {
        ESP_LOGD(TAG, "Allocating DMA buffer #%d, size=%d", i, buf_size);
        s_dma_buf[i] = (uint32_t*) malloc(buf_size);
        if (s_dma_buf[i] == NULL) {
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGV(TAG, "dma_buf[%d]=%p", i, s_dma_buf[i]);

        s_dma_desc[i].length = buf_size;     // size of a single DMA buf
        s_dma_desc[i].size = buf_size;       // total size of the chain
        s_dma_desc[i].owner = 1;
        s_dma_desc[i].sosf = 1;
        s_dma_desc[i].buf = (uint8_t*) s_dma_buf[i];
        s_dma_desc[i].offset = i;
        s_dma_desc[i].empty = 0;
        s_dma_desc[i].eof = 1;
        s_dma_desc[i].qe.stqe_next = NULL;
    }
    return ESP_OK;
}

static inline void i2s_conf_reset()
{
    const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M
            | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
    I2S0.conf.val |= conf_reset_flags;
    I2S0.conf.val &= ~conf_reset_flags;
    while (I2S0.state.rx_fifo_reset_back) {
        ;
    }
}

static void i2s_init()
{
    // Configure input GPIOs
    gpio_num_t pins[] = { s_config.pin_d7, s_config.pin_d6, s_config.pin_d5,
            s_config.pin_d4, s_config.pin_d3, s_config.pin_d2, s_config.pin_d1,
            s_config.pin_d0, s_config.pin_vsync, s_config.pin_href,
            s_config.pin_pclk };
    gpio_config_t conf = { .mode = GPIO_MODE_INPUT, .pull_up_en =
            GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE };
    for (int i = 0; i < sizeof(pins) / sizeof(gpio_num_t); ++i) {
        conf.pin_bit_mask = 1LL << pins[i];
        gpio_config(&conf);
    }

    // Route input GPIOs to I2S peripheral using GPIO matrix
    gpio_matrix_in(s_config.pin_d0, I2S0I_DATA_IN0_IDX, false);
    gpio_matrix_in(s_config.pin_d1, I2S0I_DATA_IN1_IDX, false);
    gpio_matrix_in(s_config.pin_d2, I2S0I_DATA_IN2_IDX, false);
    gpio_matrix_in(s_config.pin_d3, I2S0I_DATA_IN3_IDX, false);
    gpio_matrix_in(s_config.pin_d4, I2S0I_DATA_IN4_IDX, false);
    gpio_matrix_in(s_config.pin_d5, I2S0I_DATA_IN5_IDX, false);
    gpio_matrix_in(s_config.pin_d6, I2S0I_DATA_IN6_IDX, false);
    gpio_matrix_in(s_config.pin_d7, I2S0I_DATA_IN7_IDX, false);
    gpio_matrix_in(s_config.pin_vsync, I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(s_config.pin_href, I2S0I_H_ENABLE_IDX, false);
    gpio_matrix_in(s_config.pin_pclk, I2S0I_WS_IN_IDX, false);

    // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);
    // Toggle some reset bits in LC_CONF register
    const uint32_t lc_conf_reset_flags = I2S_IN_RST_S | I2S_AHBM_RST_S
            | I2S_AHBM_FIFO_RST_S;
    I2S0.lc_conf.val |= lc_conf_reset_flags;
    I2S0.lc_conf.val &= ~lc_conf_reset_flags;
    // Toggle some reset bits in CONF register
    i2s_conf_reset();
    // Enable slave mode (sampling clock is external)
    I2S0.conf.rx_slave_mod = 1;
    // Enable parallel mode
    I2S0.conf2.lcd_en = 1;
    // Use HSYNC/VSYNC/HREF to control sampling
    I2S0.conf2.camera_en = 1;
    // Configure clock divider
    I2S0.clkm_conf.clkm_div_a = 1;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_num = 2;
    // FIFO will sink data to DMA
    I2S0.fifo_conf.dscr_en = 1;
    // FIFO configuration, TBD if needed
    I2S0.fifo_conf.rx_fifo_mod = 1;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.conf_chan.rx_chan_mod = 1;
    // Grab 16 samples
    I2S0.sample_rate_conf.rx_bits_mod = 16;
    // Clear flags which are used in I2S serial mode
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;

    // Allocate I2S interrupt, keep it disabled
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE,
            ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
            &i2s_isr, NULL, &s_i2s_intr_handle);
}

static void i2s_fill_buf(int index)
{
    esp_intr_disable(s_i2s_intr_handle);
    i2s_conf_reset();
    I2S0.rx_eof_num = s_buf_line_width;
    I2S0.in_link.addr = (uint32_t) &s_dma_desc[index];
    I2S0.in_link.start = 1;
    I2S0.int_clr.val = I2S0.int_raw.val;
    I2S0.int_ena.in_done = 1;
    esp_intr_enable(s_i2s_intr_handle);
    I2S0.conf.rx_start = 1;
}

static void i2s_stop()
{
    esp_intr_disable(s_i2s_intr_handle);
    i2s_conf_reset();
    I2S0.conf.rx_start = 0;
    s_i2s_running = false;
}

static void i2s_run(size_t line_width, int height)
{
    s_buf_line_width = line_width;
    s_buf_height = height;

    // wait for vsync
    ESP_LOGD(TAG, "Waiting for positive edge on VSYNC");
    while (gpio_get_level(s_config.pin_vsync) == 0) {
        ;
    }
    while (gpio_get_level(s_config.pin_vsync) != 0) {
        ;
    }
    ESP_LOGD(TAG, "Got VSYNC");

    // start RX
    s_cur_buffer = 0;
    s_line_count = 0;
    s_isr_count = 0;
    s_i2s_running = true;
    i2s_fill_buf(s_cur_buffer);
}

static void IRAM_ATTR i2s_isr(void* arg)
{
    I2S0.int_clr.val = I2S0.int_raw.val;
    s_cur_buffer = !s_cur_buffer;
    if (s_isr_count == s_buf_height) {
        i2s_stop();
    } else {
        i2s_fill_buf(s_cur_buffer);
        ++s_isr_count;
    }
    BaseType_t xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(s_data_ready, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken != pdFALSE) {
        portYIELD_FROM_ISR();
    }
}

static void line_filter_task(void *pvParameters)
{
    int prev_buf = -1;
    while (true) {
        xSemaphoreTake(s_data_ready, portMAX_DELAY);
        int buf_idx = !s_cur_buffer;
        if (prev_buf != -1 && prev_buf == buf_idx) {
            ets_printf("! %d\n", s_line_count);
        }
        uint8_t* pfb = s_fb + s_line_count * s_buf_line_width;
        // Get pointer to the current DMA buffer
        const uint32_t* buf = s_dma_buf[buf_idx];
        for (int i = 0; i < s_buf_line_width; ++i) {
            // Get 32 bit from DMA buffer
            // 1 Pixel = (2Byte i2s overhead + 2Byte pixeldata)
            uint32_t v = *buf;
            // Extract third byte (only the luminance information from the pixel)
            uint8_t comp = (v & 0xff0000) >> 16;
            // Write byte to target buffer
            *pfb = comp;
            // Set source pointer 32 bit forward
            ++buf;
            // Set target pointer 8 bit forward
            ++pfb;
        }
        ++s_line_count;
        prev_buf = buf_idx;
        if (!s_i2s_running) {
            prev_buf = -1;
            xSemaphoreGive(s_frame_ready);
        }
    }
}
