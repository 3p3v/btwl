#include "SHT30Esp8266.hpp"
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

SHT30Esp8266::SHT30Esp8266() {}
SHT30Esp8266::~SHT30Esp8266() {}

SHT30Error SHT30Esp8266::sendCmd(const uint8_t msb, const uint8_t lsb) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if(i2c_master_start(cmd) != ESP_OK)
        return SHT30Err;
    if(i2c_master_write_byte(cmd, devAdd << 1 | I2C_MASTER_WRITE, SHT30_ACK_EN) != ESP_OK)
        return SHT30Err;
    if(i2c_master_write_byte(cmd, msb, SHT30_ACK_EN) != ESP_OK)
        return SHT30Err;
    if(i2c_master_write_byte(cmd, lsb, SHT30_ACK_EN) != ESP_OK)
        return SHT30Err;
    if(i2c_master_stop(cmd) != ESP_OK)
        return SHT30Err;
        
    if(i2c_master_cmd_begin(SHT30_I2C_NUM, cmd, SHT30_TIMEOUT / portTICK_PERIOD_MS) != ESP_OK)
        return SHT30Err;

    i2c_cmd_link_delete(cmd);

    ESP_LOGI("SHT30", "CMD sent");

    return SHT30Ok;
}

SHT30Error SHT30Esp8266::readData(uint8_t * data, int length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if(i2c_master_start(cmd) != ESP_OK)
        return SHT30Err;
    if(i2c_master_write_byte(cmd, devAdd << 1 | I2C_MASTER_READ, SHT30_ACK_EN) != ESP_OK)
        return SHT30Err;
    if(i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK) != ESP_OK)
        return SHT30Err;
    if(i2c_master_stop(cmd) != ESP_OK)
        return SHT30Err;

    if(i2c_master_cmd_begin(SHT30_I2C_NUM, cmd, SHT30_TIMEOUT / portTICK_PERIOD_MS) != ESP_OK)
        return SHT30Err;

    i2c_cmd_link_delete(cmd);

    ESP_LOGI("SHT30", "Data received");

    return SHT30Ok;
}