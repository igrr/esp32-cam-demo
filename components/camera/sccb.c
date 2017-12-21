/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */
#include <stdbool.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "systick.h"
#include "sccb.h"


#define SCCB_I2C_PORT   I2C_NUM_0   /* I2C port of the ESP32 to use */
#define SCCB_FREQ   (100000) // We don't need fast I2C. 100KHz is fine here.
#define TIMEOUT     (1000) /* Can't be sure when I2C routines return. Interrupts
while polling hardware may result in unknown delays. */

static const char* TAG = "sccb";

int SCCB_Init(int pin_sda, int pin_scl)
{
    i2c_config_t config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = pin_sda,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = pin_scl,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {
                    .clk_speed = SCCB_FREQ
            }
    };
    esp_err_t err = i2c_param_config(SCCB_I2C_PORT, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config: rc=0x%x", err);
        return -1;
    }
    err = i2c_driver_install(SCCB_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install: rc=0x%x", err);
        return -1;
    }
    return 0;
}

void SCCB_Deinit()
{
    i2c_driver_delete(SCCB_I2C_PORT);
}

/**
 * Scan all i2c device IDs and return the first one for which we get an ACK.
 */
uint8_t SCCB_Probe()
{
    uint8_t found_address = 0x00;

    for (uint8_t address = 0; address < 127; address++) {
        if (address > 0) {
            systick_sleep(1);
        }
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(SCCB_I2C_PORT, cmd, 100 / portTICK_PERIOD_MS);
        if (err == ESP_OK) {
            found_address = address;
            break;
        } else if (err == ESP_FAIL) {
            /* No i2c device with this address, keep looking. */
        } else {
            ESP_LOGW(TAG, "probe addr 0x%02x rc=0x%x", address, err);
        }
        i2c_cmd_link_delete(cmd);
    }

    return found_address;
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    uint8_t data = 0;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slv_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slv_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, 1);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(SCCB_I2C_PORT, cmd, TIMEOUT / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c read reg=0x%02x err=0x%x", reg, err);
        return 0xff;
    }
    return data;
}

uint8_t SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slv_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(SCCB_I2C_PORT, cmd, TIMEOUT / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c write reg=0x%02x err=0x%x", reg, err);
        return 1;
    }
    return 0;
}
