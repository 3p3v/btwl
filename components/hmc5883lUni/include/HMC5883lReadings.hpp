#pragma once
#include "stdint.h"

#define HMC5883L_ADDR 0x1e 

typedef enum {
	HMC5883lOk = 0,
	HMC5883lErr = -1,
    HMC5883lDataNotReady = -2
} HMC5883lError;

class HMC5883lReadings {
public:
    HMC5883lReadings();
    // HMC5883lReadings(uint8_t addr);
    virtual ~HMC5883lReadings() = 0;

protected:
    unsigned char devAdd;

    virtual HMC5883lError sendData(uint8_t regAddr);
    virtual HMC5883lError sendData(uint8_t regAddr, uint8_t data);
    virtual HMC5883lError sendData(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
    virtual HMC5883lError readData(uint8_t * data, int length);
};