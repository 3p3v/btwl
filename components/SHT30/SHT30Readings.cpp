#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include "SHT30Readings.hpp"
#include <esp_log.h>
#include <esp_err.h>

SHT30Storage::SHT30Storage()
    : temperature(0), humidity(0) {}

SHT30Storage::~SHT30Storage() {}

SHT30Readings::SHT30Readings() {}

SHT30Readings::~SHT30Readings() {}

Temperature SHT30Readings::getTemperatureReading() {
    return temperature;
}

Humidity SHT30Readings::getHumidityReading() {
    return humidity;
}

SHT30Cmds::SHT30Cmds() 
    : devAdd(SHT30_DEFAULT_DEV_ADD) {}

SHT30Cmds::~SHT30Cmds() {}

SHT30Error SHT30Cmds::update() {
    unsigned char data[SHT30_DATA_LEN] = {0};

    if(sendCmd(0x2C, 0x10) != SHT30Ok)
        return SHT30Err;

    vTaskDelay(SHT30_WAIT / portTICK_PERIOD_MS);

    if(readData(data, SHT30_DATA_LEN) != SHT30Ok)
        return SHT30Err;

    temperature = (*data << 8) | *(data + 1);
    humidity = (*(data + 4) << 8) | *(data + 5);
    // uint8_t crc1 = *(data + 2);
    // uint8_t crc2 = *(data + 5);

    return SHT30Ok;
}

SHT30Error SHT30Cmds::sendCmd(const uint8_t msb, const uint8_t lsb) {return SHT30Error();}

SHT30Error SHT30Cmds::readData(uint8_t * data, const int length) {return SHT30Error();}

