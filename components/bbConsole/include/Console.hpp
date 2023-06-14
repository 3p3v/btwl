#pragma once

#include "driver/uart.h"
#include "esp_log.h"
#include "string.h"
#include "BBCredentials.hpp"
#include "BBWebAddress.hpp"

#define CONSOLE_UART_NUM UART_NUM_0
#define CONSOLE_BUF_SIZE 240
#define CONSOLE_TIMEOUT 10000

typedef enum
{
    ConsoleOk = 0,
    ConsoleErr = 1,
    ConsoleExit = 2
} ConsoleError;

class Console
{
public:
    Console(BBCredentials *credentials, BBWebAddress *web_address)
        : comLen(0), credentials(credentials), web_address(web_address) {}

    /* at this point credentials and a server ip should be loaded */
    void init()
    {
        /* RTOS semaphore set */
        send_task_semaphore = xSemaphoreCreateMutex();

        /* vars set */
        memset(receivedData, 0, CONSOLE_BUF_SIZE);
        // memset(log, 0, CONSOLE_CREDENTIALS_LOG_SIZE);
        // memset(pass, 0, CONSOLE_CREDENTIALS_PASS_SIZE);
        // memset(web_address, 0,WEB_ADDRESS_WEB_ADDRESS_SIZE );

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

    ConsoleError enterConsole()
    {
        ESP_LOGI(TAG, "press ENTER to enter config mode\n");
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
                {
                    ESP_LOGI(TAG, "input not correct, resuming usual startup");
                    return ConsoleExit;
                }

                comLen = 0;

                return ConsoleOk;
            }
            default:
                ESP_LOGI(TAG, "input not correct, resuming usual startup");
                return ConsoleErr;
            }
        }
        else
        {
            ESP_LOGI(TAG, "user did not enter configuration mode, resuming usual startup");
            return ConsoleExit;
        }
    }

    ConsoleError enterCredentials()
    {
        bzero(receivedData, CONSOLE_BUF_SIZE);
        uart_write_bytes(CONSOLE_UART_NUM, "\n", 1);

        ESP_LOGI(TAG, "type login\n");
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
                    if ((comLen + event.size) > CONSOLE_CREDENTIALS_LOG_SIZE)
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
                            ESP_LOGI(TAG, "login correct");
                            goto go_pass;
                        }
                        else if (err == ConsoleExit)
                        {
                            return ConsoleExit;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Error, login not correct");
                            comLen = 0;
                            uart_flush_input(CONSOLE_UART_NUM);
                            bzero(receivedData, CONSOLE_BUF_SIZE);
                        }

                        comLen = 0;
                        uart_flush_input(CONSOLE_UART_NUM);
                        bzero(receivedData, CONSOLE_BUF_SIZE);
                    }
                    break;
                }
                default:
                    ESP_LOGI(TAG, "input not correct, resuming usual startup");
                    comLen = 0;
                    uart_flush_input(CONSOLE_UART_NUM);
                    bzero(receivedData, CONSOLE_BUF_SIZE);
                    return ConsoleExit;
                }
            }
            else
            {
                ESP_LOGI(TAG, "user did not enter configuration mode, resuming usual startup");
                comLen = 0;
                uart_flush_input(CONSOLE_UART_NUM);
                bzero(receivedData, CONSOLE_BUF_SIZE);
                return ConsoleExit;
            }
        }

    go_pass:

        comLen = 0;
        uart_flush_input(CONSOLE_UART_NUM);
        bzero(receivedData, CONSOLE_BUF_SIZE);
        ESP_LOGI(TAG, "type password");
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
                    if ((comLen + event.size) > CONSOLE_CREDENTIALS_PASS_SIZE)
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
                        ConsoleError err = pass_check();
                        if (err == ConsoleOk)
                        {
                            ESP_LOGI(TAG, "credentials correct");
                            comLen = 0;
                            uart_flush_input(CONSOLE_UART_NUM);
                            bzero(receivedData, CONSOLE_BUF_SIZE);
                            return ConsoleOk;
                        }
                        else if (err == ConsoleExit)
                        {
                            comLen = 0;
                            uart_flush_input(CONSOLE_UART_NUM);
                            bzero(receivedData, CONSOLE_BUF_SIZE);
                            return ConsoleExit;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Error, password not correct");
                            comLen = 0;
                            uart_flush_input(CONSOLE_UART_NUM);
                            bzero(receivedData, CONSOLE_BUF_SIZE);
                            return ConsoleExit;
                        }

                        comLen = 0;
                        uart_flush_input(CONSOLE_UART_NUM);
                        bzero(receivedData, CONSOLE_BUF_SIZE);
                    }
                    break;
                }
                default:
                    ESP_LOGI(TAG, "input not correct, resuming usual startup");
                    comLen = 0;
                    uart_flush_input(CONSOLE_UART_NUM);
                    bzero(receivedData, CONSOLE_BUF_SIZE);
                    return ConsoleExit;
                }
            }
            else
            {
                ESP_LOGI(TAG, "user did not enter configuration mode, resuming usual startup");
                comLen = 0;
                uart_flush_input(CONSOLE_UART_NUM);
                bzero(receivedData, CONSOLE_BUF_SIZE);
                return ConsoleExit;
            }
        }
    }

    void enterConfig()
    {
        comLen = 0;
        uart_flush_input(CONSOLE_UART_NUM);
        bzero(receivedData, CONSOLE_BUF_SIZE);
        bzero(receivedData, CONSOLE_BUF_SIZE);
        uart_write_bytes(CONSOLE_UART_NUM, "\n", 1);

        ESP_LOGI(TAG, "entering configuration menu");
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
                            comLen = 0;
                            uart_flush_input(CONSOLE_UART_NUM);
                            bzero(receivedData, CONSOLE_BUF_SIZE);
                            bzero(receivedData, CONSOLE_BUF_SIZE);
                        }
                        else if (err == ConsoleExit)
                        {
                            comLen = 0;
                            uart_flush_input(CONSOLE_UART_NUM);
                            bzero(receivedData, CONSOLE_BUF_SIZE);
                            bzero(receivedData, CONSOLE_BUF_SIZE);
                            return;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Error, flushing input");
                            comLen = 0;
                            uart_flush_input(CONSOLE_UART_NUM);
                            bzero(receivedData, CONSOLE_BUF_SIZE);
                            bzero(receivedData, CONSOLE_BUF_SIZE);
                        }

                        comLen = 0;
                        uart_flush_input(CONSOLE_UART_NUM);
                        bzero(receivedData, CONSOLE_BUF_SIZE);
                    }
                    break;
                }
                default:
                    ESP_LOGI(TAG, "input not correct, resuming usual startup");
                    comLen = 0;
                    uart_flush_input(CONSOLE_UART_NUM);
                    bzero(receivedData, CONSOLE_BUF_SIZE);
                    bzero(receivedData, CONSOLE_BUF_SIZE);
                    return;
                }
            }
            else
            {
                ESP_LOGI(TAG, "user did not enter configuration mode, resuming usual startup");
                comLen = 0;
                uart_flush_input(CONSOLE_UART_NUM);
                bzero(receivedData, CONSOLE_BUF_SIZE);
                bzero(receivedData, CONSOLE_BUF_SIZE);
                return;
            }
        }
    }

private:
    QueueHandle_t uartQueue;
    uart_event_t event;
    SemaphoreHandle_t send_task_semaphore;
    char receivedData[CONSOLE_BUF_SIZE];
    uint8_t comLen;
    BBCredentials *credentials;
    BBWebAddress *web_address;
    char TAG[8] = "CONSOLE";
    // uint8_t len;

    ConsoleError com_check()
    {
        // uint8_t len = 0;
        char *ptr = NULL;

        if (strstr(receivedData, "set url "))
        {
            ptr = strstr(receivedData, "set url ") + strlen("set url ");
            // len += strlen("set url ");
            if (web_address->setServerIP(ptr) == BBWebAddressOk)
                return ConsoleOk;
            else
                return ConsoleErr;
        }
        else if (strstr(receivedData, "set login "))
        {
            ptr = strstr(receivedData, "set login ") + strlen("set login ");
            // len += strlen("set url ");
            if (credentials->setLog(ptr) == BBWebAddressOk)
                return ConsoleOk;
            else
                return ConsoleErr;
        }
        else if (strstr(receivedData, "set password "))
        {
            ptr = strstr(receivedData, "set password ") + strlen("set password ");
            // len += strlen("set url ");
            if (credentials->setPass(ptr) == BBWebAddressOk)
                return ConsoleOk;
            else
                return ConsoleErr;
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
        if (uart_write_bytes(CONSOLE_UART_NUM, "\r", 1) != ESP_OK)
            err = ConsoleErr;
        if (uart_write_bytes(CONSOLE_UART_NUM, receivedData, comLen + len) != ESP_OK)
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
        if (uart_write_bytes(CONSOLE_UART_NUM, "\r", 1) != ESP_OK)
            err = ConsoleErr;

        uint8_t sec_len = strlen(receivedData);

        if (receivedData[sec_len - 1] == '\n')
            sec_len--;

        memset(secretData, '*', sec_len);

        if (uart_write_bytes(CONSOLE_UART_NUM, secretData, sec_len) != ESP_OK)
            err = ConsoleErr;
        // printf("%s\r", receivedData);// + comLen );

        return err;
    }

    ConsoleError log_check()
    {
        char temp[comLen + 1];
        bzero(temp, comLen);

        delete_endl(temp);

        if (strcmp(temp, credentials->getLog()))
        {
            return ConsoleOk;
        }
        else
        {
            return ConsoleErr;
        }
    }

    ConsoleError pass_check()
    {
        char temp[comLen + 1];
        bzero(temp, comLen);

        delete_endl(temp);

        if (strstr(temp, credentials->getPass()))
        {
            return ConsoleOk;
        }
        else
        {
            return ConsoleErr;
        }
    }

    ConsoleError log_set()
    {
        char temp[comLen + 1];
        bzero(temp, comLen);

        delete_endl(temp);

        if (credentials->setLog(temp) == BBCredentialsOk)
        {
            return ConsoleOk;
        }
        else
        {
            return ConsoleErr;
        }
    }

    ConsoleError pass_set()
    {
        char temp[comLen + 1];
        bzero(temp, comLen);

        delete_endl(temp);

        if (credentials->setPass(temp) == BBCredentialsOk)
        {
            return ConsoleOk;
        }
        else
        {
            return ConsoleErr;
        }
    }

    ConsoleError server_ip_set()
    {
        char temp[comLen + 1];
        bzero(temp, comLen);

        delete_endl(temp);

        if (web_address->setServerIP(temp) == BBCredentialsOk)
        {
            ESP_LOGI(TAG, "set IP to: %s\n", web_address->getServerIP());
            ESP_LOGI(TAG, "set IP to: %s\n", web_address->getWebAddress());
            return ConsoleOk;
        }
        else
        {
            ESP_LOGI(TAG, "set IP to: %s\n", web_address->getServerIP());
            ESP_LOGI(TAG, "set IP to: %s\n", web_address->getWebAddress());
            return ConsoleErr;
        }
    }

    void delete_endl(char *temp)
    {
        if (receivedData[comLen] == '\r' || receivedData[comLen] == '\n')
            memcpy(temp, receivedData, comLen - 1);
        else
            memcpy(temp, receivedData, comLen);

        // printf("%i\n", comLen);
        // printf("%s\n", temp);
    }

    ConsoleError cmd_exec(char *name, int (*func)())
    {
        char *ptr = strstr(receivedData, "set login ") + strlen("set login ");
        // len += strlen("set url ");
        if (func() == 0)
            return ConsoleOk;
        else
            return ConsoleErr;
    }
};