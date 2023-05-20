#include "HMC5883lReadings.hpp"

HMC5883lReadings::HMC5883lReadings()
    : devAdd(HMC5883L_ADDR) {}

// HMC5883lReadings::HMC5883lReadings(uint8_t addr)
//     : addr(addr) {}

HMC5883lReadings::~HMC5883lReadings() {}

HMC5883lError HMC5883lReadings::sendData(uint8_t regAddr)
{
    return HMC5883lError();
}

HMC5883lError HMC5883lReadings::sendData(uint8_t regAddr, uint8_t data) {return HMC5883lError();}

HMC5883lError HMC5883lReadings::sendData(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
    return HMC5883lError();
}

HMC5883lError HMC5883lReadings::readData(uint8_t * data, int length) {return HMC5883lError();}
