#pragma once

#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

/* GPS */
#define GPS_UART_TX                 UART_NUM_19
#define GPS_UART_RX                 UART_NUM_34
#define GPS_UART_NUM                UART_NUM_2
// #define GPS_BUF_SIZE	        	240

/* SIM800l */
#define SIM800L_UART_NUM            UART_NUM_1
#define SIM800L_UART_TX             GPIO_NUM_17
#define SIM800L_UART_RX             GPIO_NUM_16
#define SIM800L_UART_BAUD_RATE      9600
#define SIM800L_DRT_PIN             GPIO_NUM_23
#define SIM800L_RST_PIN             GPIO_NUM_18

/* I2C */
#define PIN_SDA 4
#define PIN_CLK 5

/* MPU6050 */
#define MPU6050_INT_PIN             GPIO_NUM_25

/* LID */
#define LID_LOCK_PIN                GPIO_NUM_2
#define LID_DETECTOR_PIN            GPIO_NUM_4

/* BUTTON */
#define BUTTON_INT_PIN              GPIO_NUM_27

/* MPU6050 && SHT30 */
#define MPU6050_SHT30_NUM           I2C_NUM_0

/* GPIO INT PINS */
#define TIMER_VIRTUAL_PIN           0
#define NONE_VIRTUAL_PIN            99
