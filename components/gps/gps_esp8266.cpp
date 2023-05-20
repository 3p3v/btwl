/*
 * gps_esp8266.cpp
 *
 *  Created on: Apr 7, 2023
 *      Author: esp32
 */
#include "include/gps.hpp"
#include "include/gps_esp8266.hpp"
#include "driver/uart.h"

GpsDownloaderEsp8266::GpsDownloaderEsp8266(uart_port_t uartNum, uart_config_t *uartConf, GpsUartSwapEnable swap)
	: uartNum(uartNum), swap(swap) {
	this->uartConf = uartConf;
}

GpsDownloaderEsp8266::~GpsDownloaderEsp8266() {}

GpsError GpsDownloaderEsp8266::refreshData() {
	if(swap == swapEnable)
		uart_enable_swap();
	else
		uart_disable_swap();

	uart_param_config(uartNum, uartConf);
	int len = uart_read_bytes(uartNum, (unsigned char*)data, dataLength, downloadTime / portTICK_RATE_MS);


	return 	GpsOk;
}


