#pragma once

#define LID_TIMER_T         5000     /* set timer to 5 seconds */
#define ESP_DEEP_SLEEP_T    10000
#define ESP_SEND_T          30000
#define ESP_SLEEP_CYCLES          (ESP_SEND_T / ESP_DEEP_SLEEP_T) - 1