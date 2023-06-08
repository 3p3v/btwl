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

#define GPS_DEF_UART_TX         19
#define GPS_DEF_UART_RX         34
#define GPS_DEF_UART_NUM        UART_NUM_2
#define GPS_DEF_BUF_SIZE		240

class  GpsDownloaderESP final: public GpsRawDataDownloader {
public:
	GpsDownloaderESP();
	GpsDownloaderESP(uart_port_t uartNum, int tx, int rx);
	~GpsDownloaderESP();

	virtual GpsError init();	//TODO
	virtual GpsError refreshData();

private:
	uart_port_t uartNum;
	int tx;
	int rx;
	// uart_config_t *uartConf;
};
