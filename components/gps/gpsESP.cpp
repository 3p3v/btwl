/*
 * gps_esp8266.cpp
 *
 *  Created on: Apr 7, 2023
 *      Author: esp32
 */
#include "include/gps.hpp"
#include "include/gpsESP.hpp"
#include "driver/uart.h"
#include "gpsESP.hpp"

GpsDownloaderESP::GpsDownloaderESP()
	: uartNum(GPS_DEF_UART_NUM), tx(GPS_DEF_UART_TX), rx(GPS_DEF_UART_RX) {
}

GpsDownloaderESP::GpsDownloaderESP(uart_port_t uartNum, int tx, int rx)
	: uartNum(uartNum), tx(tx), rx(rx) {}

GpsDownloaderESP::~GpsDownloaderESP() {}

GpsError GpsDownloaderESP::refreshData() {
	int len = uart_read_bytes(uartNum, (unsigned char*)data, dataLength, downloadTime / portTICK_PERIOD_MS);

	if(len <= 0)
		return GpsUnkonwnErr;
	else 
		return 	GpsOk;
}
GpsError GpsDownloaderESP::init()
{
	uart_config_t uart_config_gps = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 0,
		.source_clk = UART_SCLK_DEFAULT
	};

	if(uart_driver_install(uartNum, GPS_DEF_BUF_SIZE, GPS_DEF_BUF_SIZE, 0, NULL, 0) != ESP_OK) {
		return GpsUnkonwnErr;
	}

    if(uart_param_config(uartNum, &uartConf) != ESP_OK) {
		uart_driver_delete(uartNum);
		return GpsUnkonwnErr;
	}

	if(uart_set_pin(uartNum, tx, rx, UART_PIN_NO_CHANGE ,UART_PIN_NO_CHANGE) != ESP_OK) {
		uart_driver_delete(uartNum);
		return GpsUnkonwnErr;
	}

	return GpsOk;
}
