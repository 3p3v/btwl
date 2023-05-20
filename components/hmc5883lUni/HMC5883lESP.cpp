#include "HMC5883lESP.hpp"
#include "driver/i2c.h"
#include <esp_log.h>

HMC5883lESP::HMC5883lESP() 
    : sda(HMC5883L_DEF_I2C_SDA), scl(HMC5883L_DEF_I2C_SCL), port(HMC5883L_DEF_I2C_NUM) {}

HMC5883lESP::HMC5883lESP(int sda, int scl, i2c_port_t port) 
    : sda(sda), scl(scl), port(port) {}

HMC5883lESP::~HMC5883lESP() {}

//TODO
HMC5883lError HMC5883lESP::init(int sda, int scl, i2c_port_t port)
{
    this->sda = sda;
    this->scl = scl;
    this->port = port;

    return HMC5883lOk;
}

/* AKA select register */
HMC5883lError HMC5883lESP::sendData(uint8_t regAddr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if(i2c_master_start(cmd) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_write_byte(cmd, devAdd << 1 | I2C_MASTER_WRITE, HMC5883L_ACK_EN) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_write_byte(cmd, regAddr, HMC5883L_ACK_EN) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_stop(cmd) != ESP_OK)
        return HMC5883lErr;
        
    if(i2c_master_cmd_begin(port, cmd, HMC5883L_TIMEOUT / portTICK_PERIOD_MS) != ESP_OK)
        return HMC5883lErr;

    i2c_cmd_link_delete(cmd);

    // ESP_LOGI("HMC5883L", "CMD sent");

    return HMC5883lOk;
}

HMC5883lError HMC5883lESP::sendData(uint8_t regAddr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if(i2c_master_start(cmd) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_write_byte(cmd, devAdd << 1 | I2C_MASTER_WRITE, HMC5883L_ACK_EN) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_write_byte(cmd, regAddr, HMC5883L_ACK_EN) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_write_byte(cmd, data, HMC5883L_ACK_EN) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_stop(cmd) != ESP_OK)
        return HMC5883lErr;
        
    if(i2c_master_cmd_begin(port, cmd, HMC5883L_TIMEOUT / portTICK_PERIOD_MS) != ESP_OK)
        return HMC5883lErr;

    i2c_cmd_link_delete(cmd);

    // ESP_LOGI("HMC5883L", "CMD sent");

    return HMC5883lOk;
}

HMC5883lError HMC5883lESP::sendData(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if(i2c_master_start(cmd) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_write_byte(cmd, devAdd << 1 | I2C_MASTER_WRITE, HMC5883L_ACK_EN) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_write_byte(cmd, regAddr, HMC5883L_ACK_EN) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_write(cmd, data, length, HMC5883L_ACK_EN) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_stop(cmd) != ESP_OK)
        return HMC5883lErr;
        
    if(i2c_master_cmd_begin(port, cmd, HMC5883L_TIMEOUT / portTICK_PERIOD_MS) != ESP_OK)
        return HMC5883lErr;

    i2c_cmd_link_delete(cmd);

    // ESP_LOGI("HMC5883L", "CMD sent");

    return HMC5883lOk;
}

HMC5883lError HMC5883lESP::readData(uint8_t *data, int length)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if(i2c_master_start(cmd) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_write_byte(cmd, devAdd << 1 | I2C_MASTER_READ, HMC5883L_ACK_EN) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK) != ESP_OK)
        return HMC5883lErr;
    if(i2c_master_stop(cmd) != ESP_OK)
        return HMC5883lErr;

    if(i2c_master_cmd_begin(port, cmd, HMC5883L_TIMEOUT / portTICK_PERIOD_MS) != ESP_OK)
        return HMC5883lErr;

    i2c_cmd_link_delete(cmd);

    // ESP_LOGI("HMC5883L", "Data received");

    return HMC5883lOk;
}
