#pragma once

#include "SHT30.hpp"

#define SHT30_I2C_NUM I2C_NUM_0

class SHT30Esp8266 final : public SHT30 {
public:
    SHT30Esp8266();
    ~SHT30Esp8266();

    virtual SHT30Error sendCmd(const uint8_t msb, const uint8_t lsb) override;
    virtual SHT30Error readData(uint8_t * data, int length) override;
};