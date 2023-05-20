#pragma once

#include <stdint.h>

#define SHT30_TIMEOUT         1000
#define SHT30_WAIT            32

#define SHT30_DEFAULT_DEV_ADD 0x44
#define SHT30_ACK_EN          true
#define SHT30_TEMP_LEN        2
#define SHT30_HUMN_LEN        2
#define SHT30_CRC_LEN         1
#define SHT30_DATA_LEN        SHT30_TEMP_LEN + SHT30_HUMN_LEN + 2 * SHT30_CRC_LEN

// typedef enum {
// 	CRCOk = 0,
// 	CRCErr = 1
// } CRCError;

typedef enum {
	SHT30Ok = 0,
	SHT30Err = 1
} SHT30Error;

// class CRC {
// public:
//     int calcCRC();
//     CRCError checkCRC();
// };

typedef int Temperature;
typedef int Humidity;

class SHT30Storage {
public:
    SHT30Storage();
    virtual ~SHT30Storage() = 0;

    Temperature temperature;
    Humidity humidity;
};

class SHT30Readings : private virtual SHT30Storage {
public:
    SHT30Readings();
    virtual ~SHT30Readings() = 0;

protected:
    virtual Temperature getTemperatureReading();
    virtual Humidity getHumidityReading();

};

class SHT30Cmds : private virtual SHT30Storage {
public:
    SHT30Cmds();
    virtual ~SHT30Cmds() = 0;

    /* SETTINGS */


    /* DATA */
    SHT30Error update();

protected:
    const unsigned char devAdd;
    // CRC crc;

    /* Communication */
    virtual SHT30Error sendCmd(const uint8_t msb, const uint8_t lsb);
    virtual SHT30Error readData(uint8_t * data, const int length);
};