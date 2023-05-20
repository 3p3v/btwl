#include "HMC5883l.hpp"
#include "stdint.h"
#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <esp_err.h>

static const float hmc5883lGain [] = {
    [HMC5883L_GAIN_1370] = 0.73,
    [HMC5883L_GAIN_1090] = 0.92,
    [HMC5883L_GAIN_820]  = 1.22,
    [HMC5883L_GAIN_660]  = 1.52,
    [HMC5883L_GAIN_440]  = 2.27,
    [HMC5883L_GAIN_390]  = 2.56,
    [HMC5883L_GAIN_330]  = 3.03,
    [HMC5883L_GAIN_230]  = 4.35
};

HMC5883l::HMC5883l()
    : x(0), y(0), z(0), gain(HMC5883L_GAIN_1090) {}

HMC5883l::~HMC5883l() {}

//TODO
HMC5883lError HMC5883l::init()
{
    return HMC5883lOk;
}


HMC5883lError HMC5883l::setMode(int mode)
{
    uint8_t data = 0;
    
    if(sendData(HMC5883L_MR) != HMC5883lOk) 
        return HMC5883lErr;

    if(readData(&data, 1) != HMC5883lOk)
        return HMC5883lErr;

    // printf("%i\n", (int)data);

    data = data & (~ 0b11);
    data = data | (mode & 0b11);
    
    return sendData(HMC5883L_MR, mode);
}

HMC5883lError HMC5883l::setAveraging(int averaging)
{
    uint8_t data = 0;
    
    if(sendData(HMC5883L_CR_A) != HMC5883lOk) 
        return HMC5883lErr;

    if(readData(&data, 1) != HMC5883lOk)
        return HMC5883lErr;

    // vTaskDelay(1000);

    printf("%i\n", (int)data);

    data = data & (~(0b11 << HMC5883L_CRA5));
    data = data | ((averaging & 0b11) << HMC5883L_CRA5);

    // vTaskDelay(1000);

    printf("%i\n", (int)data);

    return sendData(HMC5883L_CR_A, data);
}

HMC5883lError HMC5883l::setDataRate(int rate)
{
    uint8_t data = 0;
    
    if(sendData(HMC5883L_CR_A) != HMC5883lOk) 
        return HMC5883lErr;

    if(readData(&data, 1) != HMC5883lOk)
        return HMC5883lErr;

    data = data & (~(0b111 << HMC5883L_CRA2));
    data = data | ((rate & 0b111) << HMC5883L_CRA2);

    return sendData(HMC5883L_CR_A, data);
}

HMC5883lError HMC5883l::setGain(int gain)
{
    uint8_t data = 0 | ((gain & 0b111) << HMC5883L_CRA5);

    if(sendData(HMC5883L_CR_A, data) == HMC5883lOk) {
        this->gain = hmc5883lGain[gain];
        return HMC5883lOk;
    } else {
        return HMC5883lErr;
    }
}

HMC5883lError HMC5883l::update()
{
    uint8_t rdy = 0;

    if(sendData(HMC5883L_SR) != HMC5883lOk) {
        ESP_LOGE("HMC5883L_SR write","");
        return HMC5883lErr;
    }

    if(readData(&rdy, 1) != HMC5883lOk) {
        ESP_LOGE("HMC5883L_SR read","");
        return HMC5883lErr;
    }

    if((rdy & (0b1)) != 1) {
        ESP_LOGE("HMC5883L_SR","not ready");
        return HMC5883lDataNotReady;
    }
    
    uint8_t data[HMC5883L_XYZ_SIZE] = {};
    
    if(sendData(HMC5883L_X_MSB) != HMC5883lOk) {
        ESP_LOGE("HMC5883L_X_MSB write","");
        return HMC5883lErr;
    }

    if(readData(data, HMC5883L_XYZ_SIZE) != HMC5883lOk) {
        ESP_LOGE("HMC5883L_X_MSB read","");
        return HMC5883lErr;
    }

    int16_t x = ((int16_t)data[HMC5883L_X_MSB - HMC5883L_X_MSB] << 8) | data[HMC5883L_X_LSB - HMC5883L_X_MSB];
    int16_t y = ((int16_t)data[HMC5883L_Y_MSB - HMC5883L_X_MSB] << 8) | data[HMC5883L_Y_LSB - HMC5883L_X_MSB];
    int16_t z = ((int16_t)data[HMC5883L_Z_MSB - HMC5883L_X_MSB] << 8) | data[HMC5883L_Z_LSB - HMC5883L_X_MSB];

    this->x = x * gain / 10;
    this->y = y * gain / 10;
    this->z = z * gain / 10;

    return HMC5883lOk;
}

float HMC5883l::getX()
{
    return x;
}

float HMC5883l::getY()
{
    return y;
}

float HMC5883l::getZ()
{
    return z;
}
