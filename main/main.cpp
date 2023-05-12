#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
// #include "driver/gpio.h"
#include <esp_log.h>
#include <esp_err.h>
#include "hal/uart_hal.h"
#include "Sim800lESP.hpp"
#include <string.h>

#define BUF_SIZE        (512)

TaskHandle_t sim_task_handle = NULL;

// static intr_handle_t handle_console;

// static void IRAM_ATTR uart_sim_isr(void *arg) {

// }

// void IRAM_ATTR  uart_sim_isr(void * arg) {
//     // int status = UART1.int_st.val;
//     // int rx_fifo_len = UART1.status.rxfifo_cnt;

//     // //UART0.fifo.rw_byte

//     // uart_clear_intr_status(UART_NUM_1, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);

//     xTaskResumeFromISR(sim_task_handle);
// }

void sim_task(void * arg) {
    uart_config_t uart_config_gps = {
	    .baud_rate = 9600,
	    .data_bits = UART_DATA_8_BITS,
	    .parity    = UART_PARITY_DISABLE,
	    .stop_bits = UART_STOP_BITS_1,
	    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT
	};

    Sim800lESP sim = Sim800lESP(uart_config_gps, "google.com");
    ESP_LOGI("SIM:", "starting init");
    sim.init();

    Sim800lError err = Sim800lErr;

    while((err != Sim800lOk)) {
        ESP_LOGI("SIM:", "starting read");
        err = sim.handshake();
    }
    err = Sim800lErr;
    
    // ESP_LOGI("SIM","%i, %s", len, ans);
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
    




    while(1) {
        char data[1000] = {};
        ESP_LOGI("SIM:", "sending http post");
        sim.sendHTTPPOST("google.com", "{\"test\": 1}");
        ESP_LOGI("SIM","%s", data);

        vTaskDelay(1000 / portTICK_PERIOD_MS);

        //vTaskSuspend(NULL);

        // char data[300] = {};
        // int len = 0;
        // //uart_get_buffered_data_len(UART_NUM_1, (size_t *)len);
        // len = uart_read_bytes(UART_NUM_1, (unsigned char*)data, 300, 1000 / portTICK_PERIOD_MS);
        // uart_flush(UART_NUM_1); 

        // for(int i = 0; i < len; i++)
        //     printf("%c", data[i]);
        // //ESP_LOGI("GPS", "%i, %s\n", len, data);
    }
}

#ifdef __cplusplus
extern "C"{
#endif
void app_main(void)
{
    // uart_driver_install(UART_NUM_1, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);

	

    // uart_param_config(UART_NUM_1, &uart_config_gps);
    // uart_set_pin(UART_NUM_1, 17, 16, UART_PIN_NO_CHANGE ,UART_PIN_NO_CHANGE);

    // uart_intr_config_t uart_intr_config_sim = {
    //     .intr_enable_mask = UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL,
    //     .rx_timeout_thresh = 20, 
    //     .txfifo_empty_intr_thresh = 20, 
    //     .rxfifo_full_thresh = 10
    // };
    // uart_intr_config(UART_NUM_1, &uart_intr_config_sim);

    // //uart_isr_free(UART_NUM_1);
    // // uart_isr_register(UART_NUM_1, uart_sim_isr, NULL);
    // uart_enable_rx_intr(UART_NUM_1);
    //esp_intr_alloc()

    /* Run tasks */
    xTaskCreate(sim_task, "sim_task", 8000, NULL, 10, &sim_task_handle);

}
#ifdef __cplusplus
}
#endif