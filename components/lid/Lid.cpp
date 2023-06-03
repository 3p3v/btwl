#include "include/Lid.hpp"
#include <esp_err.h>
#include "esp_log.h"

Lid::Lid() {
    this->lockPin = LID_DEFAULT_LOCK_PIN;
    this->detectorPin = LID_DEFAULT_DETECTOR_PIN;

    this->lockOpen = false;
    // this->detectorStatus = true;
}

Lid::Lid(gpio_num_t lock, gpio_num_t detector) {
    this->lockPin = lock;
    this->detectorPin = detector;

    this->lockOpen = false;
    // this->detectorStatus = true;
}

bool Lid::init() {
    gpio_config_t lockConfig = {
        .pin_bit_mask = BIT64(lockPin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&lockConfig));
    setLockOpen(false);

    gpio_config_t detectorkConfig = {
        .pin_bit_mask = BIT64(detectorPin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&detectorkConfig));
    // getDetectorStatus();

    return LidOk;
}

bool Lid::getDetectorOpen() {
    if(gpio_get_level(detectorPin) == 1) 
        return true;
    else    
        return false;
}

bool Lid::setLockOpen(bool status) {
    switch(status) {
        case true: {
            gpio_set_level(lockPin, 1);
            lockOpen = true;
            break;
        }
        case false: {
            gpio_set_level(lockPin, 0);
            lockOpen = false;
            break;
        }
    }

    return lockOpen;
}

bool Lid::getLockOpen() {
    return lockOpen;
}