#include "SHT30Parser.hpp"
#include <cmath>
#include <esp_log.h>
#include <esp_err.h>

SHT30Parser::SHT30Parser() {}

SHT30Parser::~SHT30Parser() {}

// SHT30Float SHT30Parser::getCalculated(float scaledRead) {
//     SHT30Float temp = SHT30Float();

//     if(scaledRead < 0) {
//         temp.positive = false;
//         scaledRead *= -1;
//     }

//     temp.sign = (int)scaledRead;
//     ESP_LOGI("SHT30", "sign: %i\n", temp.sign);
//     temp.unsign = (int)((float)(scaledRead - (float)temp.sign) * (float)pow(10, SHT30_TEMP_NSIGN_POINT));
//     ESP_LOGI("SHT30", "unsign: %i\n", temp.unsign);

//     return temp;
// }

SHT30Float SHT30Parser::getTemperature() {
    float reading = -45 + (175 * (float)getTemperatureReading() / ((pow(2, 16)) - 1));

    return reading;
}

SHT30Float SHT30Parser::getHumidity() {
    float reading = 100 * (float)getHumidityReading() / ((pow(2, 16)) - 1);

    return reading;
}