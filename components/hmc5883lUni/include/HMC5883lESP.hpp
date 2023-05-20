#pragma once

#include "driver/i2c.h"
#include "HMC5883l.hpp"

#define HMC5883L_DEF_I2C_SDA         21
#define HMC5883L_DEF_I2C_SCL         22
#define HMC5883L_DEF_I2C_NUM         I2C_NUM_0
#define HMC5883L_ACK_EN              true
#define HMC5883L_TIMEOUT             300

class HMC5883lESP final : public HMC5883l {
public:
    HMC5883lESP();
    HMC5883lESP(int sda, int scl, i2c_port_t port);
    //HMC5883lESP(uint8_t addr, int sda, int scl, i2c_port_t port);
    virtual ~HMC5883lESP();

    HMC5883lError init(int sda, int scl, i2c_port_t port);

protected:
    int sda;
    int scl;
    i2c_port_t port;

    virtual HMC5883lError sendData(uint8_t regAddr) override;
    virtual HMC5883lError sendData(uint8_t regAddr, uint8_t data) override;
    virtual HMC5883lError sendData(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) override;
    virtual HMC5883lError readData(uint8_t * data, int length) override;
};

