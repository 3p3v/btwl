#pragma once

#include "HMC5883l.hpp"

#define HMC5883L_A_POW_MATRIX_SIZE 3 * 3    /* pow(A, -1) */
#define HMC5883L_B_MATRIX_SIZE 3 * 3        

class HMC5883lCalibrated : public HMC5883l {
    HMC5883lCalibrated();
    ~HMC5883lCalibrated() = 0;

    virtual void calibrate(float b[HMC5883L_B_MATRIX_SIZE], float aPow[HMC5883L_A_POW_MATRIX_SIZE]) final;

    virtual float getX() final;
    virtual float getY() final;
    virtual float getZ() final;

private:
    float b[HMC5883L_B_MATRIX_SIZE]; 
    float aPow[HMC5883L_A_POW_MATRIX_SIZE];

}