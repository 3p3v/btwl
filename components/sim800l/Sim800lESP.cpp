#include "Sim800lESP.hpp"
// #include "Sim800lBB.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"

static QueueHandle_t sim800l_queue;

Sim800lESP::Sim800lESP(uart_config_t & uartConf, const char * url) 
    : Sim800lBB(url){
    this->uartConf = uartConf;
}

Sim800lESP::~Sim800lESP() {}

//TODO
Sim800lError Sim800lESP::init()
{
    uart_driver_install(SIM800L_DEF_UART_NUM, SIM800L_DEF_BUF_SIZE * 2, SIM800L_DEF_BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(SIM800L_DEF_UART_NUM, &uartConf);
    uart_set_pin(SIM800L_DEF_UART_NUM, SIM800L_DEF_UART_TX, SIM800L_DEF_UART_RX, UART_PIN_NO_CHANGE ,UART_PIN_NO_CHANGE);
    // uart_enable_pattern_det_baud_intr(SIM800L_DEF_UART_NUM, '\n', )  //TODO

    return Sim800lOk;
}

//TODO
Sim800lError Sim800lESP::sendData(const char *data)
{
    int len = uart_write_bytes(SIM800L_DEF_UART_NUM, data, strlen(data));
    uart_wait_tx_done(SIM800L_DEF_UART_NUM, 100);
    return Sim800lOk;
}

//TODO
int Sim800lESP::receiveData()
{
    memset(receivedData, '\0', SIM800L_DEF_BUF_SIZE);
    receivedLen = 0;

    receivedLen = uart_read_bytes(SIM800L_DEF_UART_NUM, (unsigned char*)receivedData, SIM800L_DEF_BUF_SIZE, SIM800L_DEF_DOWNLOAD_TIME / portTICK_PERIOD_MS);

    if(receivedLen > 0)
        return receivedLen;
    else
        return (int)Sim800lErr;
}

void Sim800lESP::simDelay(int ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}
