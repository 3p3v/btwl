#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "Sim800lBB.hpp"

#define SIM800L_DEF_UART_NUM         UART_NUM_1
#define SIM800L_DEF_UART_TX          17
#define SIM800L_DEF_UART_RX          16
#define SIM800L_DEF_DOWNLOAD_TIME    5000

/* Implements UART communication */
class Sim800lESP final : public Sim800lBB {
public:
    Sim800lESP(uart_config_t & uartConf, const char * url);
    virtual ~Sim800lESP();

    virtual Sim800lError init() override;

protected:
    virtual Sim800lError sendData(const char * data) override;
    virtual int receiveData() override;
    virtual void simDelay(int ms) override;

	uart_config_t uartConf;
};