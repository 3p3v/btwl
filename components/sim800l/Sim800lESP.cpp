#include "Sim800lESP.hpp"
// #include "Sim800lBB.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"

static QueueHandle_t sim800l_queue;

//TODO delete config from constructor
Sim800lESP::Sim800lESP(const uart_config_t uartConf, const char * url) 
    : Sim800lBB(url), tx(SIM800L_DEF_UART_TX), rx(SIM800L_DEF_UART_RX), uart(SIM800L_DEF_UART_NUM) {
    this->uartConf = uartConf;
}

Sim800lESP::Sim800lESP(int tx, int rx, uart_port_t uart, const uart_config_t uartConf, const char *url)
    : Sim800lBB(url), tx(tx), rx(rx), uart(uart) {
    this->uartConf = uartConf;
}

Sim800lESP::~Sim800lESP() {}

//TODO
Sim800lError Sim800lESP::init()
{
    uart_driver_install(uart, SIM800L_DEF_BUF_SIZE, SIM800L_DEF_BUF_SIZE, SIM800L_DEF_QUEUE_SIZE, &uartQueue, 0);
    uart_param_config(uart, &uartConf);
    uart_set_pin(uart, tx, rx, UART_PIN_NO_CHANGE ,UART_PIN_NO_CHANGE);
    uart_enable_pattern_det_baud_intr(uart, '\n', 1, SIM800L_PATTERN_INTERVAL, SIM800L_MIN_POST_IDLE, SIM800L_MIN_PRE_IDLE);
    uart_pattern_queue_reset(uart_port, SIM800L_DEF_PATTERN_QUEUE_SIZE);

    return Sim800lOk;
}

//TODO
Sim800lError Sim800lESP::sendData(const char *data)
{
    int len = uart_write_bytes(uart, data, strlen(data));
    // uart_wait_tx_done(uart, 100);
    return Sim800lOk;
}

//TODO
int Sim800lESP::receiveData()
{
    
    
    memset(receivedData, '\0', SIM800L_DEF_BUF_SIZE);
    receivedLen = 0;

    receivedLen = uart_read_bytes(uart, (unsigned char*)receivedData, SIM800L_DEF_BUF_SIZE, SIM800L_DEF_DOWNLOAD_TIME / portTICK_PERIOD_MS);

    if(receivedLen > 0)
        return receivedLen;
    else
        return (int)Sim800lErr;
}

void Sim800lESP::simDelay(int ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}
