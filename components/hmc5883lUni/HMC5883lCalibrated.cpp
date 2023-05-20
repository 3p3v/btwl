#include "HMC5883lCalibrated.hpp"

HMC5883lCalibrated::HMC5883lCalibrated() {
    memset(b, 0, HMC5883L_B_MATRIX_SIZE);
    memset(aPow, 1, HMC5883L_A_POW_MATRIX_SIZE);
}

HMC5883lCalibrated::~HMC5883lCalibrated() {}

void HMC5883lCalibrated::calibrate(float b[3], float aPow[9]) {
    memcpy(this->b, b, HMC5883L_B_MATRIX_SIZE);
    memcpy(this->aPow, aPow, HMC5883L_A_POW_MATRIX_SIZE);
}

float HMC5883lCalibrated::getX()
{
    
}


