#pragma once

#include "driver/uart.h"
#include "esp_log.h"
#include "string.h"

static char TAG[] = "CONSOLE";

#define CONSOLE_UART_NUM UART_NUM_0
#define CONSOLE_BUF_SIZE 240
#define CONSOLE_TIMEOUT 10000
#define CONSOLE_LOG_SIZE 50
#define CONSOLE_PASS_SIZE 50

typedef enum
{
    ConsoleOk = 0,
    ConsoleErr = 1,
    ConsoleExit = 2
} ConsoleError;

class Console
{
public:
    Console()
        : comLen(0) {}

    void init()
    {
        /* RTOS semaphore set */
        send_task_semaphore = xSemaphoreCreateMutex();
        
        /* vars set */
        memset(receivedData, 0, CONSOLE_BUF_SIZE);
        memset(log, 0, CONSOLE_LOG_SIZE);
        memset(pass, 0, CONSOLE_PASS_SIZE);
        memset(webAddress, 0, )

        /* load login and password from nvs */
        esp_err_t err = nvs_pass(nvconsole, NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGI("CONSOLE", "Error (%s) passing NVS handle!\n", esp_err_to_name(err));
            esp_restart();
        } else {
            ESP_LOGI("CONSOLE", "reading login");
            err = nvs_get_str(nvs_handle, "log", log, CONSOLE_LOG_SIZE);
            switch (err) {
                case ESP_OK:
                    ESP_LOGI("CONSOLE", "Done\n");
                    break;
                case ESP_ERR_NVS_NOT_FOUND: {
                    ESP_LOGI("CONSOLE", "The value has not been not initialized yet!\n");
                    strcpy(log, def_log);

                    ESP_LOGI("CONSOLE", "saving");
                    err = nvs_set_str(nvs_handle, "log", log);
                    // ESP_LOGI("CONSOLE", (err != ESP_OK) ? "Failed!\n" : "Done\n");

                    ESP_LOGI("CONSOLE", "saving");
                    err = nvs_commit(nvs_handle);
                    // ESP_LOGI("CONSOLE", (err != ESP_OK) ? "Failed!\n" : "Done\n");
                    break;
                }
                default :
                    ESP_LOGI("CONSOLE", "Error (%s) reading!\n", esp_err_to_name(err));
            }

            ESP_LOGI("CONSOLE", "reading pass");
            err = nvs_get_str(nvs_handle, "pass", pass, CONSOLE_PASS_SIZE);
            switch (err) {
                case ESP_OK:
                    ESP_LOGI("CONSOLE", "Done\n");
                    break;
                case ESP_ERR_NVS_NOT_FOUND: {
                    ESP_LOGI("CONSOLE", "The value has not been not initialized yet!\n");
                    strcpy(pass, def_pass);

                    ESP_LOGI("CONSOLE", "saving");
                    err = nvs_set_str(nvs_handle, "pass", pass);
                    // ESP_LOGI("CONSOLE", (err != ESP_OK) ? "Failed!\n" : "Done\n");

                    ESP_LOGI("CONSOLE", "saving");
                    err = nvs_commit(nvs_handle);
                    // ESP_LOGI("CONSOLE", (err != ESP_OK) ? "Failed!\n" : "Done\n");
                    break;
                }
                default :
                    ESP_LOGI("CONSOLE", "Error (%s) reading!\n", esp_err_to_name(err));
            }

            nvs_close(nvs_handle);
        }

        /* load server ip form nvs *///TODO move to webaddress class
        esp_err_t err = nvs_pass(nvadd, NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGI("CONSOLE", "Error (%s) passing NVS handle!\n", esp_err_to_name(err));
            esp_restart();
        } else {
            ESP_LOGI("CONSOLE", "reading server ip");
            err = nvs_get_str(nvs_handle, "add", webAddress, CONSOLE_LOG_SIZE);
            switch (err) {
                case ESP_OK:
                    ESP_LOGI("CONSOLE", "Done\n");
                    break;
                case ESP_ERR_NVS_NOT_FOUND: {
                    ESP_LOGI("CONSOLE", "server ip has not been not initialized yet!\n");
                    strcpy(log, def_log);

                    ESP_LOGI("CONSOLE", "saving");
                    err = nvs_set_str(nvs_handle, "log", log);
                    // ESP_LOGI("CONSOLE", (err != ESP_OK) ? "Failed!\n" : "Done\n");

                    ESP_LOGI("CONSOLE", "saving");
                    err = nvs_commit(nvs_handle);
                    // ESP_LOGI("CONSOLE", (err != ESP_OK) ? "Failed!\n" : "Done\n");
                    break;
                }
                default :
                    ESP_LOGI("CONSOLE", "Error (%s) reading!\n", esp_err_to_name(err));
            }
        }

        /* UART config */
        uart_config_t uartConf = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT};
        ESP_ERROR_CHECK(uart_driver_install(CONSOLE_UART_NUM, CONSOLE_BUF_SIZE, CONSOLE_BUF_SIZE, 10, &uartQueue, 0));
        ESP_ERROR_CHECK(uart_param_config(CONSOLE_UART_NUM, &uartConf));
        ESP_ERROR_CHECK(uart_set_pin(CONSOLE_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    }

    void enterConsole()
    {
        if (xQueueReceive(uartQueue, (void *)&event, CONSOLE_TIMEOUT / portTICK_PERIOD_MS))
        {
            bzero(receivedData, CONSOLE_BUF_SIZE);
            // ESP_LOGI(TAG, "uart[%d] event:", uart);
            switch (event.type)
            {
            // Event of UART receving data
            case UART_DATA:
            {

                comLen += uart_read_bytes(CONSOLE_UART_NUM, receivedData, event.size, portMAX_DELAY);
                printf("%s", receivedData);
                if (strstr(receivedData, "\n") || strstr(receivedData, "\r"))
                    ESP_LOGI(TAG, "entering config mode");
                else
                    ESP_LOGI(TAG, "input not correct, resuming usual startup");

                comLen = 0;

                break;
            }
            default:
                ESP_LOGI(TAG, "input not correct, resuming usual startup");
                // break;
            }
        }
        else
        {
            ESP_LOGI(TAG, "user did not enter configuration mode, resuming usual startup");
            return;
        }
    }

    void enterCredentials()
    {
        bzero(receivedData, CONSOLE_BUF_SIZE);
        uart_write_bytes(CONSOLE_UART_NUM, "\n", 1);

        while (1)
        {
            if (xQueueReceive(uartQueue, (void *)&event, portMAX_DELAY))
            {

                // ESP_LOGI(TAG, "uart[%d] event:", uart);
                switch (event.type)
                {
                // Event of UART receving data
                case UART_DATA:
                {
                    if ((comLen + event.size) > CONSOLE_LOG_SIZE)
                    {
                        ESP_LOGI(TAG, "login too long, flushing input");
                        uart_flush_input(CONSOLE_UART_NUM);
                        bzero(receivedData, CONSOLE_BUF_SIZE);
                        comLen = 0;
                        break;
                    }

                    con_draw();

                    if (strstr(receivedData, "\n") || strstr(receivedData, "\r"))
                    {
                        uart_write_bytes(CONSOLE_UART_NUM, "\n", 1);
                        ConsoleError err = log_check();
                        if (err == ConsoleOk)
                        {
                            goto: go_pass;
                        }
                        else if (err == ConsoleExit)
                        {
                            return;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Error, login not correct");
                        }

                        comLen = 0;
                        uart_flush_input(CONSOLE_UART_NUM);
                        bzero(receivedData, CONSOLE_BUF_SIZE);
                    }
                    break;
                }
                default:
                    ESP_LOGI(TAG, "input not correct, resuming usual startup");
                }
            }
            else
            {
                ESP_LOGI(TAG, "user did not enter configuration mode, resuming usual startup");
                return;
            }
        }

        go_pass:
    }

    void enterConfig()
    {
        bzero(receivedData, CONSOLE_BUF_SIZE);
        uart_write_bytes(CONSOLE_UART_NUM, "\n", 1);

        while (1)
        {
            if (xQueueReceive(uartQueue, (void *)&event, portMAX_DELAY))
            {

                // ESP_LOGI(TAG, "uart[%d] event:", uart);
                switch (event.type)
                {
                // Event of UART receving data
                case UART_DATA:
                {
                    if ((comLen + event.size) > CONSOLE_BUF_SIZE)
                    {
                        ESP_LOGI(TAG, "command too long, flushing input");
                        uart_flush_input(CONSOLE_UART_NUM);
                        bzero(receivedData, CONSOLE_BUF_SIZE);
                        comLen = 0;
                        break;
                    }

                    con_draw();

                    if (strstr(receivedData, "\n") || strstr(receivedData, "\r"))
                    {
                        uart_write_bytes(CONSOLE_UART_NUM, "\n", 1);
                        ConsoleError err = com_check();
                        if (err == ConsoleOk)
                        {
                            ESP_LOGI(TAG, "Done");
                        }
                        else if (err == ConsoleExit)
                        {
                            return;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Error, flushing input");
                        }

                        comLen = 0;
                        uart_flush_input(CONSOLE_UART_NUM);
                        bzero(receivedData, CONSOLE_BUF_SIZE);
                    }
                    break;
                }
                default:
                    ESP_LOGI(TAG, "input not correct, resuming usual startup");
                }
            }
            else
            {
                ESP_LOGI(TAG, "user did not enter configuration mode, resuming usual startup");
                return;
            }
        }
    }

private:
    QueueHandle_t uartQueue;
    uart_event_t event;
    SemaphoreHandle_t send_task_semaphore;
    nvs_handle_t nvs_handle;
    char receivedData[CONSOLE_BUF_SIZE];
    uint8_t comLen;
    char log[CONSOLE_LOG_SIZE];
    char pass[CONSOLE_PASS_SIZE];
    const char def_log[] = "default";
    const char def_pass[] = "default";
    const char def_add[] = "3.215.18.200";
    const char nvconsole[] = "nvconsole";
    const char nvadd[] = "nvadd";
    // uint8_t len;

    ConsoleError com_check()
    {
        if (strstr(receivedData, "set url "))
        {
            return ConsoleOk;
        }
        else if (strstr(receivedData, "exit"))
        {
            ESP_LOGI(TAG, "exiting config mode");
            return ConsoleExit;
        }
        else
        {
            ESP_LOGI(TAG, "command not recognised");
            return ConsoleErr;
        }
    }

    ConsoleError con_draw()
    {
        ConsoleError err = ConsoleOk;
        uint8_t len = uart_read_bytes(CONSOLE_UART_NUM, receivedData + comLen, event.size, portMAX_DELAY);
        // ESP_LOGI(TAG, "[UART DATA len]: %d", comLen);
        if(uart_write_bytes(CONSOLE_UART_NUM, "\r", 1) !=ESP_OK)
            err = ConsoleErr;
        if(uart_write_bytes(CONSOLE_UART_NUM, receivedData, comLen + len) !=ESP_OK)
            err = ConsoleErr;
        // printf("%s\r", receivedData);// + comLen );
        comLen += len;

        return err;
    }

    ConsoleError sec_draw()
    {
        char secretData[CONSOLE_BUF_SIZE];
        bzero(receivedData, CONSOLE_BUF_SIZE);
        
        ConsoleError err = ConsoleOk;
        comLen += uart_read_bytes(CONSOLE_UART_NUM, receivedData + comLen, event.size, portMAX_DELAY);
        // ESP_LOGI(TAG, "[UART DATA len]: %d", comLen);
        if(uart_write_bytes(CONSOLE_UART_NUM, "\r", 1) !=ESP_OK)
            err = ConsoleErr;

        uint8_t sec_len = strlen(receivedData);

        if(receivedData[sec_len - 1] == '\n')
            sec_len--;

        memset(secretData, '*', sec_len);

        if(uart_write_bytes(CONSOLE_UART_NUM, secretData, sec_len) !=ESP_OK)
            err = ConsoleErr;
        // printf("%s\r", receivedData);// + comLen );

        return err;
    }

    ConsoleError log_check() {

    }

     ConsoleError pass_check() {
        
    }
};