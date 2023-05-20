#pragma once

#include "HMC5883lReadings.hpp"

#define HMC5883L_DEF_ADD 0x1e
#define HMC5883L_DEF_ID 0x00333448

#define HMC5883L_CR_A   0x00  /* Configuration Register A */
#define HMC5883L_CR_B   0x01  /* Configuration Register B */
#define HMC5883L_MR     0x02  /* Mode Register */
#define HMC5883L_X_MSB  0x03  
#define HMC5883L_X_LSB  0x04
#define HMC5883L_Y_MSB  0x05
#define HMC5883L_Y_LSB  0x06
#define HMC5883L_Z_MSB  0x07
#define HMC5883L_Z_LSB  0x08
#define HMC5883L_SR     0x09
#define HMC5883L_IR_A   0x0a
#define HMC5883L_IR_B   0x0b
#define HMC5883L_IR_C   0x0c

#define HMC5883L_XYZ_SIZE    6

typedef enum {
    HMC5883L_CRA0 = 0,
    HMC5883L_CRA1 = 1,
    HMC5883L_CRA2 = 2,
    HMC5883L_CRA3 = 3,
    HMC5883L_CRA4 = 4,
    HMC5883L_CRA5 = 5,
    HMC5883L_CRA6 = 6,
    HMC5883L_CRA7 = 7
} HMC5883lCRA;

/**
 * Device operating mode
 */
typedef enum
{
    HMC5883L_MODE_CONTINUOUS = 0, //!< Continuous mode
    HMC5883L_MODE_SINGLE          //!< Single measurement mode, default
} hmc5883l_opmode_t;

/**
 * Number of samples averaged per measurement
 */
typedef enum
{
    HMC5883L_SAMPLES_1 = 0, //!< 1 sample, default
    HMC5883L_SAMPLES_2,     //!< 2 samples
    HMC5883L_SAMPLES_4,     //!< 4 samples
    HMC5883L_SAMPLES_8      //!< 8 samples
} hmc5883l_samples_averaged_t;

/**
 * Data output rate in continuous measurement mode
 */
typedef enum
{
    HMC5883L_DATA_RATE_00_75 = 0, //!< 0.75 Hz
    HMC5883L_DATA_RATE_01_50,     //!< 1.5 Hz
    HMC5883L_DATA_RATE_03_00,     //!< 3 Hz
    HMC5883L_DATA_RATE_07_50,     //!< 7.5 Hz
    HMC5883L_DATA_RATE_15_00,     //!< 15 Hz, default
    HMC5883L_DATA_RATE_30_00,     //!< 30 Hz
    HMC5883L_DATA_RATE_75_00,     //!< 75 Hz
    HMC5883L_DATA_RATE_220_00     //!< 220 Hz, HMC5983 only
} hmc5883l_data_rate_t;

/**
 * Measurement mode of the device (bias)
 */
typedef enum
{
    HMC5883L_BIAS_NORMAL = 0, //!< Default flow, no bias
    HMC5883L_BIAS_POSITIVE,   //!< Positive bias configuration all axes, used for self test (see datasheet)
    HMC5883L_BIAS_NEGATIVE    //!< Negative bias configuration all axes, used for self test (see datasheet)
} hmc5883l_bias_t;

/**
 * Device gain
 */
typedef enum
{
    HMC5883L_GAIN_1370 = 0, //!< 0.73 mG/LSb, range -0.88..+0.88 G
    HMC5883L_GAIN_1090,     //!< 0.92 mG/LSb, range -1.3..+1.3 G, default
    HMC5883L_GAIN_820,      //!< 1.22 mG/LSb, range -1.9..+1.9 G
    HMC5883L_GAIN_660,      //!< 1.52 mG/LSb, range -2.5..+2.5 G
    HMC5883L_GAIN_440,      //!< 2.27 mG/LSb, range -4.0..+4.0 G
    HMC5883L_GAIN_390,      //!< 2.56 mG/LSb, range -4.7..+4.7 G
    HMC5883L_GAIN_330,      //!< 3.03 mG/LSb, range -5.6..+5.6 G
    HMC5883L_GAIN_230       //!< 4.35 mG/LSb, range -8.1..+8.1 G
} hmc5883l_gain_t;

class HMC5883l : public HMC5883lReadings {
public:
    HMC5883l();
    // HMC5883l(uint8_t addr);
    virtual ~HMC5883l() = 0;

    virtual HMC5883lError init() final;

    virtual HMC5883lError setMode(int mode) final;
    virtual HMC5883lError setAveraging(int averaging) final;
    virtual HMC5883lError setDataRate(int rate) final;
    virtual HMC5883lError setGain(int gain) final;
    virtual HMC5883lError update() final;

    virtual float getX();
    virtual float getY();
    virtual float getZ();

protected:
    /* in micro-Teslas */
    float x;
    float y;
    float z;

    float gain;
};