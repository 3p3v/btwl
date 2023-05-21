#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "Sim800lBB.hpp"
#include "driver/gpio.h"

/* UART config */
#define SIM800L_DEF_UART_NUM            UART_NUM_1
#define SIM800L_DEF_UART_TX             17
#define SIM800L_DEF_UART_RX             16
#define SIM800L_DEF_UART_BAUD_RATE      9600

/* Sleep and reset pins */
#define SIM800L_DEF_DRT_PIN             23
#define SIM800L_DEF_RST_PIN             18

/* Interrupt config */
#define SIM800L_DEF_DOWNLOAD_TIME       5000
#define SIM800L_DEF_QUEUE_SIZE          10
#define SIM800L_DEF_PATTERN_QUEUE_SIZE  5
#define SIM800L_PATTERN_INTERVAL        0
#define SIM800L_MIN_POST_IDLE           0
#define SIM800L_MIN_PRE_IDLE            0

/* Implements UART communication */
class Sim800lESP final : public Sim800lBB {
public:
    Sim800lESP(const char *url);
    Sim800lESP(gpio_num_t tx, gpio_num_t rx, uart_port_t uart, gpio_num_t drt, gpio_num_t rst, const int baudRate, const char *url);
    virtual ~Sim800lESP();

    virtual Sim800lError init() override;

protected:
    virtual Sim800lError sendData(const char * data) override;
    virtual int receiveData() override;
    virtual void simDelay(int ms) override;

    /* Set DRT pin high/low */
    virtual Sim800lError setDRT(Sim800lPin set);
    /* Set RST pin high/low */
    virtual Sim800lError setRST(Sim800lPin set);

    gpio_num_t tx; 
    gpio_num_t rx;
    int baudRate;
    gpio_num_t drt;
    gpio_num_t rst;
    uart_port_t uart;
	uart_config_t uartConf;
    QueueHandle_t uartQueue;
};