#pragma once

#include "SHT30Readings.hpp"
#include "BBFormats.hpp"

#define SHT30_TEMP_NSIGN_POINT   3
#define SHT30_HUM_NSIGN_POINT    3
// #define SHT30_TEMP_NSIGN_POINT   3
// #define SHT30_HUM_NSIGN_POINT    3

typedef float SHT30Float;

class SHT30Parser : private SHT30Readings {
public:
    SHT30Parser();
    virtual ~SHT30Parser() = 0;

    SHT30Float getTemperature();
    SHT30Float getHumidity();

private:
    SHT30Float getCalculated(float scaledRead);
};