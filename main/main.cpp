#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include <esp_log.h>
#include <esp_err.h>
#include "hal/uart_hal.h"
#include "Sim800lESP.hpp"
#include <string.h>
#include "HMC5883lCalibartion.hpp"
#include "gps.hpp"
#include "gpsESP.hpp"
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "sdkconfig.h"
#include "MPU6050Parser.hpp"
#include <JsonData.hpp>
#include "SHT30Esp8266.hpp"

#define PIN_SDA 4
#define PIN_CLK 5

TaskHandle_t sim_task_handle = NULL;
TaskHandle_t acc_task_handle = NULL;

/* MPU6050 */
MPU6050 mpu = MPU6050();
MPU6050Parser mpuParser = MPU6050Parser(&mpu, 32768, 2);
/* SHT30 */
SHT30Esp8266 sht = SHT30Esp8266();

void acc_task(void * arg) {


	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)HMC5883L_DEF_I2C_SDA;
	conf.scl_io_num = (gpio_num_t)HMC5883L_DEF_I2C_SCL;
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 120, 120, 0));

	vTaskDelay(200 / portTICK_PERIOD_MS);

    /* MPU6050 init */
	mpu.initialize();
    mpu.setWakeCycleEnabled(true);
    /* SHT30 init */
    //not implemented

    while(1) {
        /* MPU6050 */
		mpuParser.update();
        sht.update();

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void sim_task(void * arg) {

    /* UART init */
    uart_config_t uart_config_gps = {
	    .baud_rate = 9600,
	    .data_bits = UART_DATA_8_BITS,
	    .parity    = UART_PARITY_DISABLE,
	    .stop_bits = UART_STOP_BITS_1,
	    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT
	};

    /* SIM800l */
    Sim800lESP sim = Sim800lESP(uart_config_gps, "google.com");
    /* NEO-6m */
    GpsDownloaderESP downloader(&uart_config_gps);
	GpsCmdsFinder finder(&downloader);
	GpsAllData gps = GpsAllData(&finder, &downloader);
	GpsGGA gga(&finder);
	gps.add(&gga);
	/* SHT30 */
	SHT30Esp8266 sht = SHT30Esp8266();

    /* SIM800l init*/
    ESP_LOGI("SIM:", "starting init");
    sim.init();

    Sim800lError err = Sim800lErr;

    while((err != Sim800lOk)) {
        ESP_LOGI("SIM:", "starting read");
        err = sim.handshake();
    }
    err = Sim800lErr;
    
    while((err != Sim800lOk)) {
        ESP_LOGI("SIM:", "setting gprs");
        err = sim.setConnectionType("GPRS");
    }
    err = Sim800lErr;

    while((err != Sim800lOk)) {
        ESP_LOGI("SIM:", "starting plus");
        err = sim.setAccessPoint("Plus");
    }
    err = Sim800lErr;
    
    /* GPS init */
    ESP_LOGI("GPS:", "starting");
    downloader.init();

    while(1) {
        /* NEO-6m */
		gps.update();

        /* SHT30 */
		SHT30Error shtErr =  sht.update();

        /* Get output data from acc */
        //not implemented
        mpuParser.update();

        /* Make new JSON */
        JsonData json = JsonData();
		json.init();
		json.addEspMac();
		json.addGpsInfo(gga.updateErr, (double)gga.getLatitude(), (double)gga.getLongitude());
		json.addAccInfo(0, (double)mpuParser.getAccX(), (double)mpuParser.getAccY(), (double)mpuParser.getAccZ());
		json.addTempInfo(shtErr, (double)sht.getTemperature(), (double)sht.getHumidity());
        json.addTime(gga.getTime());
        json.addBatteryInfo(0,100);
		char * jsonString = json.getJsonData();
        ESP_LOGI("JSON", "%s\n", jsonString);
        // ESP_LOGI("JSON", "%s\n", "{\"jsonString\": 1}");

        /* Send POST */
        ESP_LOGI("SIM:", "sending http post");
        sim.sendHTTPPOST("ec2-3-235-240-194.compute-1.amazonaws.com/telemetries", jsonString);//"{\n\"tt\":\t1\n}");

        // vTaskSuspend(NULL);
    }
}

#ifdef __cplusplus
extern "C"{
#endif
void app_main(void)
{
    xTaskCreatePinnedToCore(sim_task, "sim_task", 12000, NULL, 10, &sim_task_handle, 0);
    xTaskCreate(acc_task, "acc_task", 12000, NULL, 10, &acc_task_handle);
}
#ifdef __cplusplus
}
#endif