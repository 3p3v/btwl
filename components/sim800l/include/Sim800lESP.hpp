#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "Sim800lBB.hpp"

/* UART config */
#define SIM800L_DEF_UART_NUM            UART_NUM_1
#define SIM800L_DEF_UART_TX             17
#define SIM800L_DEF_UART_RX             16

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
    Sim800lESP(const uart_config_t uartConf, const char *url);
    Sim800lESP(int tx, int rx, uart_port_t uart, const uart_config_t uartConf, const char *url);
    virtual ~Sim800lESP();

    virtual Sim800lError init() override;

protected:
    virtual Sim800lError sendData(const char * data) override;
    virtual int receiveData() override;
    virtual void simDelay(int ms) override;

    int tx; 
    int rx;
    uart_port_t uart;
	uart_config_t uartConf;
    QueueHandle_t uartQueue;
};