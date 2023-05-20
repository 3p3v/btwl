#pragma once
#include "HMC5883lESP.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void hmc5883l_calibarte_task() {

    

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)HMC5883L_DEF_I2C_SDA;
	conf.scl_io_num = (gpio_num_t)HMC5883L_DEF_I2C_SCL;
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;
	// conf.clk_stretch_tick = 300;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));

    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 120, 120, 0));

    HMC5883lESP hmc = HMC5883lESP();
    
    // hmc.init();

    hmc.setMode((int)HMC5883L_MODE_SINGLE);
    hmc.setAveraging((int)HMC5883L_SAMPLES_8);
    hmc.setDataRate((int)HMC5883L_DATA_RATE_75_00);
    hmc.setGain((int)HMC5883L_GAIN_1090);

    while(1) {
        hmc.setMode((int)HMC5883L_MODE_SINGLE);
        hmc.update();

        printf("%f,%f,%f\r\n", hmc.getX(), hmc.getY(), hmc.getZ());
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}