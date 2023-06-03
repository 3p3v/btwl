#pragma once

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

#define LID_DEFAULT_LOCK_PIN        GPIO_NUM_2
#define LID_DEFAULT_DETECTOR_PIN    GPIO_NUM_4

typedef enum {
    LidOk = 0,
    LidErr = 1
} LidErrStatus;

// typedef enum {
//     LidDetectorOpen = 1;
//     LidDetectorClosed = 2;
//     // LidDetectorErr = 3
// } LidDetectorStatus;

// typedef enum {
//     LidLockOpen = 1;
//     LidLockClosed = 2;
//     // LidLockErr = 3
// } LidLockStatus;

class Lid {
public:
    Lid();
    Lid(gpio_num_t lock, gpio_num_t detector);

    bool init();

    /* Use in new task */
    bool getDetectorOpen();

    /* Setter for setting lock after getting  */
    bool setLockOpen(bool status);

    /* Getter */
    bool getLockOpen();
    

protected:
    gpio_num_t lockPin;
    gpio_num_t detectorPin;

    bool lockOpen;
    // LidDetectorStatus detectorStatus;
};