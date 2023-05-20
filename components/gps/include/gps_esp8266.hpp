/*
 * gps_esp8266.hpp
 *
 *  Created on: Apr 7, 2023
 *      Author: esp32
 */

#pragma once

#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "driver/uart.h"

typedef enum {
	swapEnable = 0,
	swapDisable = 1
} GpsUartSwapEnable;

class  GpsDownloaderEsp8266 final: public GpsRawDataDownloader {
public:
	GpsDownloaderEsp8266(uart_port_t uartNum, uart_config_t *uartConf, GpsUartSwapEnable swap);
	~GpsDownloaderEsp8266();

	virtual GpsError init();	//TODO
	virtual GpsError refreshData();

private:
	uart_port_t uartNum;
	GpsUartSwapEnable swap;
	uart_config_t *uartConf;
};
