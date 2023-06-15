#pragma once

#define LID_TIMER_T         5000     /* set timer to 5 seconds */
#define LID_ALARM_TIMER_T   10000
#define ACCEL_ALARM_TIMER_T 10000
#define BUTTON_TIMER_T      10000

#define ESP_DEEP_SLEEP_T    10000
#define ESP_SEND_T          30000
#define ESP_SLEEP_CYCLES          (ESP_SEND_T / ESP_DEEP_SLEEP_T)

#define BB_MAX_ADDRESS_LEN  150