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
#include "Lid.hpp"
#include "Message.hpp"
#include "pins_def.hpp"
#include "esp_timer.h"
#include "other_def.hpp"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "nvs_flash.h"
#include "Console.hpp"

/* DEFINITIONS */
char *webAddress;

/* HANDLERS */
TaskHandle_t sim_task_handle = NULL;
TaskHandle_t acc_task_handle = NULL;
TaskHandle_t ans_send_task_handle = NULL;
TaskHandle_t telemetry_send_task_handle = NULL;
TaskHandle_t idle_get_task_handle = NULL;
TaskHandle_t gpio_task_handle = NULL;
esp_timer_handle_t lid_timer;
esp_timer_handle_t send_timer;
esp_timer_handle_t lid_alarm_timer;
esp_timer_handle_t accel_alarm_timer;
esp_timer_handle_t button_timer;

/* SEMAPHORES */
SemaphoreHandle_t send_task_semaphore = NULL;
SemaphoreHandle_t message_update_semaphore = NULL;
SemaphoreHandle_t sim_task_semaphore = NULL;

/* QUEUES */
QueueHandle_t gpio_evt_queue = NULL;

/* MODULES INITIALIZATION */
JsonData json = JsonData();
/* MPU6050 */
MPU6050 mpu = MPU6050();
MPU6050Parser mpuParser = MPU6050Parser(&mpu, 32768, 2);
/* SHT30 */
SHT30Esp8266 sht = SHT30Esp8266();
/* SIM800l */
Sim800lESP sim = Sim800lESP("google.com");
/* NEO-6m */
GpsDownloaderESP downloader = GpsDownloaderESP();
GpsCmdsFinder finder(&downloader);
GpsGGA gga(&finder);
GpsAllData gps = GpsAllData(&finder, &downloader);
/* Lid */
Lid lid;
/* Messages */
ESPNonVolatileMessage currentMessage(false, false, true);
ServerMessage pendingMessage(false, false, true, false);
/* Web Addres */
BBWebAddress bbWebAddress = BBWebAddress();

/* FUNCTION'S DECLARATIONS */
static void reloadSimTask();
static void acc_task(void *arg);
static void sim_task(void *arg);
static void telemetry_send_task(void *arg);
static void idle_get_task(void *arg);

/* VARS */
static bool lid_timer_on = false;
static bool accel_alarm_timer_on = false;
static bool lid_alarm_timer_on = false;
static bool lid_alarm_triggered = false;
static bool button_timer_on = false;
static bool lid_alarm_fired = true;
static bool lid_alarm_in_progress = false;
static RTC_DATA_ATTR uint8_t esp_wake_cycle;

static void lock_accel_alarm()
{
    ESP_ERROR_CHECK(gpio_intr_disable(MPU6050_INT_PIN));
    accel_alarm_timer_on = true;
    esp_timer_start_once(accel_alarm_timer, ACCEL_ALARM_TIMER_T * 1000);
}

static void unlock_accel_alarm()
{
    ESP_ERROR_CHECK(gpio_intr_enable(MPU6050_INT_PIN));
    accel_alarm_timer_on = false;
}

static void lock_lid_alarm()
{
    // ESP_ERROR_CHECK(gpio_intr_disable(LID_DETECTOR_PIN));
    lid_alarm_timer_on = true;
    lid_alarm_triggered = true;
    esp_timer_start_once(lid_alarm_timer, LID_ALARM_TIMER_T * 1000);
    ESP_DRAM_LOGI("lid_alarm", "lock_lid_alarm");
}

static void unlock_lid_alarm()
{
    // ESP_ERROR_CHECK(gpio_intr_enable(LID_DETECTOR_PIN));
    ESP_DRAM_LOGI("lid_alarm", "unlock_lid_alarm");
    lid_alarm_fired = true;
    lid_alarm_timer_on = false;
}

static void lock_button()
{
    ESP_ERROR_CHECK(gpio_intr_disable(BUTTON_INT_PIN));
    button_timer_on = true;
    esp_timer_start_once(button_timer, BUTTON_TIMER_T * 1000);
}

static void unlock_button()
{
    ESP_ERROR_CHECK(gpio_intr_enable(BUTTON_INT_PIN));
    button_timer_on = false;
}

/* GPIO interrupt handler */
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;

    if (gpio_num == BUTTON_INT_PIN)
    {
        esp_wake_cycle = 0;
        lock_button();
        ESP_DRAM_LOGI("GPIO_ISR", "BUTTON_INT_PIN", (int)arg);
    }
    else if (gpio_num == MPU6050_INT_PIN)
    {
        esp_wake_cycle = 0;
        lock_accel_alarm();
        ESP_DRAM_LOGI("GPIO_ISR", "MPU6050_INT_PIN", (int)arg);
        mpuParser.update();
    }
    else if (gpio_num == LID_DETECTOR_PIN)
    {
        esp_wake_cycle = 0;
        // gpio_intr_disable(LID_DETECTOR_PIN);
        // lock_accel_alarm();
        ESP_DRAM_LOGI("GPIO_ISR", "LID_DETECTOR_PIN");
    }
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/* lid timer callback */
static void lid_timer_callback(void *arg)
{
    /* Update message */
    // if (xSemaphoreTake(message_update_semaphore, portMAX_DELAY))
    // {
    currentMessage.setOpen(false);
    int pin = LID_T_VIRTUAL_PIN;
    lid_timer_on = false;
    xQueueSendFromISR(gpio_evt_queue, &pin, NULL);
    //     xSemaphoreGive(message_update_semaphore);
    // }
}

/* lid timer callback
 * reqiest new get/post after certain time
 */
static void send_timer_callback(void *arg)
{
    ESP_LOGI("TIMER_CALLBACK", "requesting idle task");
    int pin = TIMER_VIRTUAL_PIN;
    xQueueSend(gpio_evt_queue, &pin, NULL);
    esp_wake_cycle = 0;
}

/* lid alarm timer callback */
static void lid_alarm_timer_callback(void *arg)
{
    ESP_DRAM_LOGI("lid_alarm", "unlock_lid_alarm");
    ESP_LOGI("LID_ALARM_TIMER_CALLBACK", "unlocking alarm");
    unlock_lid_alarm();
}

/* acceleration alarm timer callback */
static void accel_alarm_timer_callback(void *arg)
{
    ESP_LOGI("ACCEL_ALARM_TIMER_CALLBACK", "unlocking alarm");
    unlock_accel_alarm();
}

/* button timer callback */
static void button_timer_callback(void *arg)
{
    ESP_LOGI("BUTTON_TIMER_CALLBACK", "unlocking alarm");
    unlock_button();
}

/* Reload sim task */
// static void reloadSimTask()
// {
//     sim_task_handle = NULL;
//     xTaskCreate(sim_task, "sim_task", 12000, NULL, 10, &sim_task_handle);
//     vTaskDelete(NULL);
// }

static void update_message()
{
    if (xSemaphoreTake(message_update_semaphore, portMAX_DELAY))
    {
        ESP_LOGI("JSON", "protect: %d", json.getProtect());
        ESP_LOGI("JSON", "open: %d", json.getOpen());
        ESP_LOGI("JSON", "ack: %d", json.getAck());
        pendingMessage.setValid(true);
        pendingMessage.setProtect(json.getProtect());
        pendingMessage.setOpen(json.getOpen());
        pendingMessage.setAck(json.getAck());
    }
    xSemaphoreGive(message_update_semaphore);
}

static void telemetry_send_task(void *arg)
{
    if (xSemaphoreTake(send_task_semaphore, portMAX_DELAY))
    {
        if (sim.getSleep() == true)
        {
            sim.sleepModeDisable();
            sim.commonSettings();
        }

        /* Make new JSON */
        char output[SIM800L_DEF_BUF_SIZE];
        memset(output, 0, SIM800L_DEF_BUF_SIZE);

        json.initNewMessage();

        // json.addEspMac();
        json.addGpsInfo(gga.updateErr, (double)gga.getLatitude(), (double)gga.getLongitude());
        json.addAccInfo(0, (double)mpuParser.getAccX(), (double)mpuParser.getAccY(), (double)mpuParser.getAccZ());
        json.addTempInfo(/*shtErr*/ 0, (double)sht.getTemperature(), (double)sht.getHumidity());
        json.addTime(gga.getTime());
        json.addBatteryInfo(0, 100);
        char *jsonString = json.getJsonData();
        ESP_LOGI("JSON", "%s\n", jsonString);

        /* Send POST */
        /* Set AP */ // TODO do better error handling
        ESP_LOGI("SIM", "Sending HTTP POST.");
        char address[100];
        memset(address, 0, 100);
        sprintf(address, "%s/telemetries", webAddress);
        Sim800lError err = sim.sendHTTPPOST(address, jsonString, output); //"{\n\"tt\":\t1\n}");
        if (err == Sim800lRecErr)
        {
            ESP_LOGI("SIM", "Sending HTTP POST failed.");
            // TODO save and send later
        }
        else if (err == Sim800lTimeoutErr || err == Sim800lErr)
        {
            ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
            sim.resetForce();
            // reloadSimTask();
            esp_restart();
        }
        else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr)
        {
            ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
            sim.reinit();
            // reloadSimTask();
            esp_restart();
        }
        else if (err == Sim800lOk)
            ESP_LOGI("SIM", "Sending HTTP POST OK.");

        json.deinitNewMessage();

        if (json.update(output) == JsonDataOk)
        { // TODO move to JsonData
            update_message();
        }

        xSemaphoreGive(send_task_semaphore);

        esp_timer_stop(send_timer);
        esp_timer_start_once(send_timer, ESP_SEND_T * 1000);

        int pin = TASK_END_VIRTUAL_PIN;
        xQueueSend(gpio_evt_queue, &pin, NULL);

        vTaskDelete(NULL);
    }
}

static void idle_get_task(void *arg)
{
    if (xSemaphoreTake(send_task_semaphore, portMAX_DELAY))
    {
        if (sim.getSleep() == true)
        {
            sim.sleepModeDisable();
            sim.commonSettings();
        }

        char output[SIM800L_DEF_BUF_SIZE];
        memset(output, 0, SIM800L_DEF_BUF_SIZE);

        /* Send GET */
        /* Set AP */ // TODO do better error handling
        ESP_LOGI("SIM", "Sending HTTP GET1.");
        char address[100];
        memset(address, 0, 100);
        sprintf(address, "%s/idle", webAddress);
        ESP_LOGI("SIM", "Sending HTTP GET2.");
        Sim800lError err = sim.sendHTTPGET(address, output); //"{\n\"tt\":\t1\n}");
        if (err == Sim800lRecErr)
        {
            ESP_LOGI("SIM", "Sending HTTP GET failed.");
            // TODO save and send later
        }
        else if (err == Sim800lTimeoutErr || err == Sim800lErr)
        {
            ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
            sim.resetForce();
            // reloadSimTask();
            esp_restart();
        }
        else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr)
        {
            ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
            sim.reinit();
            // reloadSimTask();
            esp_restart();
        }
        else if (err == Sim800lOk)
            ESP_LOGI("SIM", "Sending HTTP GET OK.");

        // json.update(output);
        if (json.update(output) == JsonDataOk)
        { // TODO move to JsonData
            update_message();
        }

        xSemaphoreGive(send_task_semaphore);

        esp_timer_stop(send_timer);
        esp_timer_start_once(send_timer, ESP_SEND_T * 1000);

        int pin = TASK_END_VIRTUAL_PIN;
        xQueueSend(gpio_evt_queue, &pin, NULL);

        vTaskDelete(NULL);
    }
}

static void ans_send_task(void *arg)
{
    if (xSemaphoreTake(send_task_semaphore, portMAX_DELAY))
    {
        if (sim.getSleep() == true)
        {
            sim.sleepModeDisable();
            sim.commonSettings();
        }

        /* Make new JSON */
        char output[SIM800L_DEF_BUF_SIZE];
        memset(output, 0, SIM800L_DEF_BUF_SIZE);
        json.initNewMessage();

        json.addMessage(currentMessage);

        char *jsonString = json.getJsonData();
        ESP_LOGI("JSON", "%s\n", jsonString);

        /* Send POST */
        /* Set AP */ // TODO do better error handling
        ESP_LOGI("SIM", "Sending HTTP POST.");
        char address[100];
        memset(address, 0, 100);
        sprintf(address, "%s/answer", webAddress);
        Sim800lError err = sim.sendHTTPPOST(address, jsonString, output); //"{\n\"tt\":\t1\n}");
        if (err == Sim800lRecErr)
        {
            ESP_LOGI("SIM", "Sending HTTP POST failed.");
            // TODO save and send later
        }
        else if (err == Sim800lTimeoutErr || err == Sim800lErr)
        {
            ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
            sim.resetForce();
            // reloadSimTask();
            esp_restart();
        }
        else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr)
        {
            ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
            sim.reinit();
            // reloadSimTask();
            esp_restart();
        }
        else if (err == Sim800lOk)
            ESP_LOGI("SIM", "Sending HTTP POST OK.");

        // json.update(output);
        if (json.update(output) == JsonDataOk)
        { // TODO move to JsonData
            update_message();
        }

        json.deinitNewMessage();
    }

    xSemaphoreGive(send_task_semaphore);

    int pin = TASK_END_VIRTUAL_PIN;
    xQueueSend(gpio_evt_queue, &pin, NULL);

    vTaskDelete(NULL);
}

static void alarm_send_task(void *arg)
{
    if (xSemaphoreTake(send_task_semaphore, portMAX_DELAY))
    {
        if (sim.getSleep() == true)
        {
            sim.sleepModeDisable();
            sim.commonSettings();
        }

        /* Make new JSON */
        char output[SIM800L_DEF_BUF_SIZE];
        memset(output, 0, SIM800L_DEF_BUF_SIZE);
        json.initNewMessage();

        int *code = (int *)arg;
        json.addAlarm(*code);

        char *jsonString = json.getJsonData();
        ESP_LOGI("JSON", "%s\n", jsonString);

        /* Send POST */
        /* Set AP */ // TODO do better error handling
        ESP_LOGI("SIM", "Sending HTTP POST.");
        char address[100];
        memset(address, 0, 100);
        sprintf(address, "%s/alarms", webAddress);
        Sim800lError err = sim.sendHTTPPOST(address, jsonString, output); //"{\n\"tt\":\t1\n}");
        if (err == Sim800lRecErr)
        {
            ESP_LOGI("SIM", "Sending HTTP POST failed.");
            // TODO save and send later
        }
        else if (err == Sim800lTimeoutErr || err == Sim800lErr)
        {
            ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
            sim.resetForce();
            // reloadSimTask();
            esp_restart();
        }
        else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr)
        {
            ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
            sim.reinit();
            // reloadSimTask();
            esp_restart();
        }
        else if (err == Sim800lOk)
            ESP_LOGI("SIM", "Sending HTTP POST OK.");

        // json.update(output);
        if (json.update(output) == JsonDataOk)
        { // TODO move to JsonData
            update_message();
        }

        json.deinitNewMessage();
    }

    xSemaphoreGive(send_task_semaphore);

    int pin = TASK_END_VIRTUAL_PIN;
    xQueueSend(gpio_evt_queue, &pin, NULL);

    vTaskDelete(NULL);
}

/* Getting and transforming data && choosing correct messages and requests */
static void acc_task(void *arg)
{
    uint32_t io_num = NONE_VIRTUAL_PIN;

    xQueueReceive(gpio_evt_queue, &io_num, 0 / portTICK_PERIOD_MS);
    // do
    while (1)
    {
        ESP_LOGI("MAIN", "io_num = %i", (int)io_num);

        switch ((int)io_num)
        {
        case MPU6050_INT_PIN:
        {
            if (currentMessage.getOpen() == false && accel_alarm_timer_on == false)
            {
                /* MPU6050 update */
                // mpuParser.update();
                /* max acceleretion exceeded, send alarm to server */
                /* send telemetry */
                ESP_LOGI("MAIN", "starting telemetry alarm task");
                if (currentMessage.getProtect() == true)
                    xTaskCreate(telemetry_send_task, "telemetry_send_task", 12000, NULL, 10, &telemetry_send_task_handle);
                else
                {
                    ESP_LOGI("MAIN", "starting idle task");
                    xTaskCreate(idle_get_task, "idle_get_task", 12000, NULL, 10, &idle_get_task_handle);
                }
                /* send alarm */
                lock_accel_alarm();
                ESP_LOGI("ALARM", "max acceleretion exceeded, sending alarm to server");
                static int code = 2;
                xTaskCreate(alarm_send_task, "alarm_send_task", 12000, &code, 10, &idle_get_task_handle);
                break;
            }
            else
            {
                ESP_LOGI("MAIN", "protect is not on");
                break;
            }
        }
        case LID_DETECTOR_PIN:
        {
            lid_alarm_triggered = true;
            lid_alarm_in_progress = true;

            if (lid_timer_on == true)
            {
                lid_timer_on = false;
                esp_timer_stop(lid_timer);
            }
            if (lid_alarm_timer_on == false && lid_alarm_fired == true)
            {
                lid_alarm_fired = false;
                lock_lid_alarm();
                /* send telemetry */
                ESP_LOGI("MAIN", "starting telemetry alarm task");
                if (currentMessage.getProtect() == true)
                    xTaskCreate(telemetry_send_task, "telemetry_send_task", 12000, NULL, 10, &telemetry_send_task_handle);
                else
                {
                    ESP_LOGI("MAIN", "starting idle task");
                    xTaskCreate(idle_get_task, "idle_get_task", 12000, NULL, 10, &idle_get_task_handle);
                }

                ESP_LOGI("ALARM", "lid should be closed, sending alarm to server");
                /* Send alarm to server */
                static int code = 1;
                xTaskCreate(alarm_send_task, "alarm_send_task", 12000, &code, 10, &idle_get_task_handle);
            }
        }
        case NONE_VIRTUAL_PIN:
        {
            break;
        }
        case TASK_END_VIRTUAL_PIN:
        {
            break;
        }
        case LID_T_VIRTUAL_PIN:
        {
            if (xSemaphoreTake(message_update_semaphore, portMAX_DELAY))
            {
                currentMessage.setOpen(false);

                xSemaphoreGive(message_update_semaphore);
            }
            break;
        }
        case BUTTON_INT_PIN:
        case TIMER_VIRTUAL_PIN:
        {
            ESP_LOGI("MAIN", "waiting to start idle task");
            if (currentMessage.getProtect() == true)
                xTaskCreate(telemetry_send_task, "telemetry_send_task", 12000, NULL, 10, &telemetry_send_task_handle);
            else
            {
                ESP_LOGI("MAIN", "starting idle task");
                xTaskCreate(idle_get_task, "idle_get_task", 35000, NULL, 10, &idle_get_task_handle);
            }
            break;
        }
        }

        /* SHT30 */
        sht.update();
        /* NEO-6m */
        gps.update();
        /* Lid */
        ESP_LOGI("LID", "detector %d", lid.getDetectorOpen());
        ESP_LOGI("LID", "lock %d", lid.getLockOpen());
        ESP_LOGI("LID", "current protect %d", currentMessage.getProtect());
        ESP_LOGI("LID", "current open %d", currentMessage.getOpen());
        ESP_LOGI("LID", "current ack %d", currentMessage.getAck());
        ESP_LOGI("LID", "pending protect %d", pendingMessage.getProtect());
        ESP_LOGI("LID", "pending open %d", pendingMessage.getOpen());
        ESP_LOGI("LID", "pending ack %d", pendingMessage.getAck());
        /* No new message has come, only check if the box works properly*/
        if (xSemaphoreTake(message_update_semaphore, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI("LID", "checking...");
            if (currentMessage.isSameWithAck(pendingMessage) == true && currentMessage.getAck() == true) // && pendingMessage.getValid() == true)
            {
                ESP_LOGI("MAIN", "mesages same");
                /* Lid must be opened */
                if (currentMessage.getOpen() == true)
                {
                    ESP_LOGI("LID", "lid should be open");
                    if (lid.getDetectorOpen() == true)
                    {
                        lid.setLockOpen(false);
                    }
                    else
                    {
                        lid.setLockOpen(true);
                    }
                }
                /* Lid must be closed */
                else
                {
                    ESP_LOGI("ALARM_LID", "lid should be closed");
                    lid.setLockOpen(false);
                    if (lid.getDetectorOpen() == true)
                    {
                        lid_alarm_triggered = true;
                        lid_alarm_in_progress = true;

                        if (lid_timer_on == true)
                        {
                            lid_timer_on = false;
                            esp_timer_stop(lid_timer);
                        }
                        if (lid_alarm_timer_on == false && lid_alarm_fired == true)
                        {
                            lid_alarm_fired = false;
                            lock_lid_alarm();
                            ESP_LOGI("ALARM", "lid should be closed, sending alarm to server");
                            /* Send alarm to server */
                            static int code = 1;
                            xTaskCreate(alarm_send_task, "alarm_send_task", 12000, &code, 10, &idle_get_task_handle);
                        }
                    }
                    else
                    {
                        /* check if in previous iteration it was opened while it shouldn't */
                        if (lid_alarm_triggered == true && lid_timer_on == false)
                        {
                            lid_alarm_triggered = false;
                            lid_timer_on = true;
                            esp_timer_start_once(lid_timer, LID_TIMER_T * 1000);
                        }
                        else if (lid_alarm_triggered == false && lid_timer_on == false && lid_alarm_in_progress == true)
                        {
                            lid_alarm_in_progress = false;
                            esp_timer_stop(lid_alarm_timer);
                            unlock_lid_alarm();
                            ESP_LOGI("ALARM", "lid was closed");
                            /* Send alarm to server */
                            static int code = 0;
                            xTaskCreate(alarm_send_task, "alarm_send_task", 12000, &code, 10, &idle_get_task_handle);
                        }
                    }
                }
            }
            /* Something changed */
            else if (currentMessage.isSame(pendingMessage) == false)
            {
                /* Lid lock change */
                if (currentMessage.getOpen() != pendingMessage.getOpen())
                {
                    /* Set open */
                    if (pendingMessage.getOpen() == true)
                    {
                        currentMessage.setOpen(true);

                        if (lid.getDetectorOpen() == true)
                        {
                            lid.setLockOpen(false);
                        }
                        else
                        {
                            lid.setLockOpen(true);
                        }
                    }
                    /* Set close */
                    else if (pendingMessage.getOpen() == false)
                    {
                        lid.setLockOpen(false);
                        if (lid.getDetectorOpen() == true)
                        {
                            /* stop lid_timer */
                            if (lid_timer_on == true)
                            {
                                lid_timer_on = false;
                                esp_timer_stop(lid_timer);
                            }
                        }
                        else
                        {
                            /* start lid_timer */
                            if (lid_timer_on == false)
                            {
                                lid_timer_on = true;
                                /* wait for LID_TIMER_T ms to be certain that the box was closed */
                                esp_timer_start_once(lid_timer, LID_TIMER_T * 1000);
                            }
                        }
                    }
                }
                /* Telemetry change */
                else if (currentMessage.getProtect() != pendingMessage.getProtect())
                {
                    if (pendingMessage.getProtect() == true)
                        currentMessage.setProtect(true);
                    else
                        currentMessage.setProtect(false);
                }
            }
            /* Everything else is ok so just send confirmation */
            if (currentMessage.isSame(pendingMessage) == true && pendingMessage.getAck() == false)
            {
                currentMessage.setAck(true);
                xTaskCreate(ans_send_task, "ans_send_task", 12000, NULL, 10, &ans_send_task_handle);
            }

            xSemaphoreGive(message_update_semaphore);
        }

        io_num = NONE_VIRTUAL_PIN;
        /* if there wasnt any interrupt or lid is not open then break */
        if (((uxQueueMessagesWaiting(gpio_evt_queue) > 0) ||                                                      /* there are some tasks to do */
             (currentMessage.isSameWithAck(pendingMessage) == false) ||                                           /* message was not updated or acknowledged */
             ((telemetry_send_task_handle != NULL) && (eTaskGetState(telemetry_send_task_handle) != eDeleted)) || /* a task is running */
             ((idle_get_task_handle != NULL) && (eTaskGetState(idle_get_task_handle) != eDeleted)) ||             /* a task is running */
             ((ans_send_task_handle != NULL) && (eTaskGetState(ans_send_task_handle) != eDeleted)) ||             /* a task is running */
             lid_timer_on == true ||                                                                              /* waiting for the lid to be closed (required to acknowledge message) */
             lid_alarm_timer_on == true ||  
             accel_alarm_timer_on == true ||                                                                        /* clear alarm first */
             lid_alarm_in_progress == true ||                                                                     /* acceleration was exceeded, wait and clear the flag */
             button_timer_on == true ||                                                                           /* button was pressed, wait and clear the flag */
             (!currentMessage.getOpen() && lid.getDetectorOpen())))                                               /* the box is opened while it shouldn't, have to send alarms or to clear an alarm when it's closed */
        {
            xQueueReceive(gpio_evt_queue, &io_num, ESP_DEEP_SLEEP_T / portTICK_PERIOD_MS);
            ESP_LOGI("MAIN", "new cycle");
            ESP_LOGI("MAIN", "queue size: %i", (int)uxQueueMessagesWaiting(gpio_evt_queue));
        }
        else
        {
            ESP_LOGI("MAIN", "exiting");
            ESP_LOGI("MAIN", "queue size: %i", (int)uxQueueMessagesWaiting(gpio_evt_queue));
            break;
        }
    }
    // } while ((xQueueReceive(gpio_evt_queue, &io_num, ESP_DEEP_SLEEP_T / portTICK_PERIOD_MS)) || (currentMessage.getAck() == false) || ((telemetry_send_task_handle != NULL) && (eTaskGetState(telemetry_send_task_handle) != eDeleted)) || ((idle_get_task_handle != NULL) && (eTaskGetState(idle_get_task_handle) != eDeleted))  || ((ans_send_task_handle != NULL) && (eTaskGetState(ans_send_task_handle) != eDeleted)));

    ESP_LOGI("MAIN", "exited");
    // } while (currentMessage.getAck() == false);

    /* Disable isrs */
    ESP_ERROR_CHECK(gpio_reset_pin(LID_DETECTOR_PIN));
    ESP_ERROR_CHECK(gpio_intr_disable(LID_DETECTOR_PIN));
    ESP_ERROR_CHECK(gpio_reset_pin(BUTTON_INT_PIN)); // TODO ????
    ESP_ERROR_CHECK(gpio_intr_disable(BUTTON_INT_PIN));
    ESP_ERROR_CHECK(gpio_reset_pin(MPU6050_INT_PIN));
    ESP_ERROR_CHECK(gpio_intr_disable(MPU6050_INT_PIN));

    /* Enable deep sleep */
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(ESP_DEEP_SLEEP_T * 1000)); // timer
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << MPU6050_INT_PIN;
    const uint64_t ext_wakeup_pin_2_mask = 1ULL << BUTTON_INT_PIN;

    if (currentMessage.getOpen() == true)
    { // ext1
        if (lid.getDetectorOpen() == true)
            ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(LID_DETECTOR_PIN, 1));
        else
            ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(LID_DETECTOR_PIN, 0));
        ESP_ERROR_CHECK(rtc_gpio_pullup_en(LID_DETECTOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(LID_DETECTOR_PIN));
    }
    else
    {
        ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(LID_DETECTOR_PIN, 0));
        ESP_ERROR_CHECK(rtc_gpio_pullup_en(LID_DETECTOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(LID_DETECTOR_PIN));
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH));
        ESP_ERROR_CHECK(esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON));
        ESP_ERROR_CHECK(rtc_gpio_pullup_dis(MPU6050_INT_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(MPU6050_INT_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pullup_dis(BUTTON_INT_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pulldown_en(BUTTON_INT_PIN));
    }

    if (sim.getSleep() == false)
        sim.sleepModeEnable();
    // vTaskDelay(100);
    ESP_LOGI("SLEEP", "START");
    esp_deep_sleep_start();
}

// static void sim_task(void *arg)
// {
//     if (xSemaphoreTake(sim_task_semaphore, portMAX_DELAY))
//     {
//         /* SIM800l init*/
//         // if (sim.getInitStatus() != Sim800lUARTInitialised)
//         // {
//         //     ESP_LOGI("SIM", "Starting initialization.");
//         //     sim.init();
//         // }

//         Sim800lError err = Sim800lErr;

//         /* SIM800l synchronization */
//         ESP_LOGI("SIM", "Starting synchronization.");
//         err = sim.handshake(10);
//         if (err == Sim800lErr || err == Sim800lTimeoutErr || err == Sim800lRecErr)
//         {
//             ESP_LOGI("SIM", "Proper response wasn't received %i times. Hardware SIM800l error. Rebooting device...", 10);
//             esp_restart();
//         }
//         else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr)
//         {
//             ESP_LOGI("SIM", "UART perypherial error. Rebooting device...");
//             esp_restart();
//         }
//         ESP_LOGI("SIM", "Synchronization OK.");

//         /* SIM card check */
//         ESP_LOGI("SIM", "Checking SIM card.");
//         err = sim.getSIMInfo();
//         if (err == Sim800lRecErr)
//         {
//             ESP_LOGI("SIM", "Sim card not inserted, cannot progress. Rebooting device...");
//             esp_restart();
//         }
//         else if (err == Sim800lTimeoutErr || err == Sim800lErr)
//         {
//             ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting device...");
//             esp_restart();
//         }
//         else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr)
//         {
//             ESP_LOGI("SIM", "UART perypherial error. Rebooting device...");
//             esp_restart();
//         }
//         ESP_LOGI("SIM", "SIM card OK.");

//         /* Network check */
//         // ESP_LOGI("SIM", "Checking if device registered to network.");
//         // for(int i = 0; i < 5; i++) {
//         //     err = sim.checkIfRegistered();
//         //     if(err == Sim800lRegistered || err == Sim800lRoamingRegistered)
//         //         break;
//         //     else if(err == Sim800lRespErr && i < 4) {
//         //         ESP_LOGI("SIM", "Cannot register, switching on and off airplane mode.");
//         //         sim.airplaneModeEnable();
//         //         vTaskDelay(100 / portTICK_PERIOD_MS);
//         //         sim.airplaneModeDisable();
//         //         vTaskDelay(2000 / portTICK_PERIOD_MS);
//         //     }
//         //     else if (err == Sim800lRespErr && i < 5) {
//         //         ESP_LOGI("SIM", "Couldn't register to network %i times. Rebooting SIM800l...", 5);
//         //         sim.resetForce();
//         //         reloadSimTask();
//         //     } else if (err == Sim800lTimeoutErr || err == Sim800lErr) {
//         //         ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
//         //         sim.resetForce();
//         //         reloadSimTask();
//         //     } else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr) {
//         //         ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
//         //         sim.reinit();
//         //         reloadSimTask();
//         //     }
//         // }

//         /* Set GPRS */
//         ESP_LOGI("SIM", "Setting GPRS.");
//         err = sim.setConnectionType("GPRS");
//         if (err == Sim800lRecErr)
//         {
//             ESP_LOGI("SIM", "GPRS could not be set, cannot progress. Rebooting SIM800l...");
//             sim.resetForce();
//             reloadSimTask();
//         }
//         else if (err == Sim800lTimeoutErr || err == Sim800lErr)
//         {
//             ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
//             sim.resetForce();
//             reloadSimTask();
//         }
//         else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr)
//         {
//             ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
//             sim.reinit();
//             reloadSimTask();
//         }
//         ESP_LOGI("SIM", "Setting GPRS OK.");

//         err = Sim800lErr;

//         /* Set AP */
//         ESP_LOGI("SIM:", "Setting AP to...");
//         err = sim.setAccessPoint("Plus");
//         if (err == Sim800lRecErr)
//         {
//             ESP_LOGI("SIM", "AP could not be set, cannot progress. Rebooting SIM800l...");
//             sim.resetForce();
//             reloadSimTask();
//         }
//         else if (err == Sim800lTimeoutErr || err == Sim800lErr)
//         {
//             ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
//             sim.resetForce();
//             reloadSimTask();
//         }
//         else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr)
//         {
//             ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
//             sim.reinit();
//             reloadSimTask();
//         }
//         ESP_LOGI("SIM", "Setting AP OK.");

//         // err = sim.writeSAPBR(1, 1);
//         // if(err != Sim800lOk) {
//         //     sim.resetForce();
//         //     reloadSimTask();
//         // }

//         // err = sim.writeSAPBR(2, 1);
//         // if(err != Sim800lOk) {
//         //     sim.resetForce();
//         //     reloadSimTask();
//         // }

//         xSemaphoreGive(sim_task_semaphore);
//     }

//     vTaskDelete(sim_task_handle);
// }

#ifdef __cplusplus
extern "C"
{
#endif
    void app_main(void)
    {
        /* initialoze non-volatile memory */
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            // NVS partition was truncated and needs to be erased
            // Retry nvs_flash_init
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);

        /* get last current message from nvs */
        currentMessage.init();
        pendingMessage.setOpen(currentMessage.getOpen());
        pendingMessage.setProtect(currentMessage.getProtect());
        pendingMessage.setAck(currentMessage.getAck());

        /* clean up after deep sleep */
        rtc_gpio_deinit(LID_DETECTOR_PIN);
        rtc_gpio_deinit(BUTTON_INT_PIN);
        rtc_gpio_deinit(MPU6050_INT_PIN);

        /* init FreeRTOS objects */                         // TODO
        send_task_semaphore = xSemaphoreCreateMutex();      // xSemaphoreCreateBinary();
        message_update_semaphore = xSemaphoreCreateMutex(); // xSemaphoreCreateBinary();
        sim_task_semaphore = xSemaphoreCreateMutex();
        gpio_evt_queue = xQueueCreate(15, sizeof(uint32_t));

        /* I2C init */
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = (gpio_num_t)HMC5883L_DEF_I2C_SDA;
        conf.scl_io_num = (gpio_num_t)HMC5883L_DEF_I2C_SCL;
        conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
        conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
        conf.master.clk_speed = 400000;
        conf.clk_flags = 0;

        ESP_ERROR_CHECK(i2c_param_config(MPU6050_SHT30_NUM, &conf));
        ESP_ERROR_CHECK(i2c_driver_install(MPU6050_SHT30_NUM, I2C_MODE_MASTER, 120, 120, 0));

        /* GPIO && GPIO int */
        const gpio_config_t config = {
            .pin_bit_mask = BIT(MPU6050_INT_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE};
        gpio_config(&config);
        gpio_set_intr_type(MPU6050_INT_PIN, GPIO_INTR_NEGEDGE);
        gpio_set_intr_type(LID_DEFAULT_DETECTOR_PIN, (gpio_int_type_t)(GPIO_INTR_ANYEDGE));
        const gpio_config_t config1 = {
            .pin_bit_mask = BIT(BUTTON_INT_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE};
        gpio_config(&config1);
        gpio_set_intr_type(BUTTON_INT_PIN, (gpio_int_type_t)GPIO_INTR_POSEDGE);
        gpio_install_isr_service(0);
        gpio_ler_add(BUisr_handler_add(MPU6050_INT_PIN, gpio_isr_handler, (void *)MPU6050_INT_PIN);
        gpio_isr_handler_add(LID_DEFAULT_DETECTOR_PIN, gpio_isr_handler, (void *)LID_DEFAULT_DETECTOR_PIN);
        gpio_isr_handTTON_INT_PIN, gpio_isr_handler, (void *)BUTTON_INT_PIN);

        /* lid timer & lid timer int config */
        const esp_timer_create_args_t lid_timer_args = {
            .callback = &lid_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .arg = (void *)lid_timer,
            .name = "lid-timer"};
        ESP_ERROR_CHECK(esp_timer_create(&lid_timer_args, &lid_timer));

        /* lid alarm timer & lid alarm timer int config */
        const esp_timer_create_args_t lid_alarm_timer_args = {
            .callback = &lid_alarm_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .arg = (void *)lid_alarm_timer,
            .name = "lid-alarm-timer"};
        ESP_ERROR_CHECK(esp_timer_create(&lid_alarm_timer_args, &lid_alarm_timer));

        /* acceleration alarm timer & acceleration alarm timer int config */
        const esp_timer_create_args_t accel_alarm_timer_args = {
            .callback = &accel_alarm_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .arg = (void *)accel_alarm_timer,
            .name = "lid-timer"};
        ESP_ERROR_CHECK(esp_timer_create(&accel_alarm_timer_args, &accel_alarm_timer));

        /* send timer & send timer int config */
        const esp_timer_create_args_t send_timer_args = {
            .callback = &send_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .arg = (void *)send_timer,
            .name = "send-timer"};
        ESP_ERROR_CHECK(esp_timer_create(&send_timer_args, &send_timer));
        // esp_timer_start_once(send_timer, ESP_SEND_T * 1000);                  //start counting in case programm got stuck

        /* objects init */
        ESP_LOGI("SIM", "Starting initialization.");
        sim.init();
        /* GPS init */
        gps.add(&gga);
        ESP_LOGI("GPS:", "starting");
        downloader.init();
        /* Lid init */
        lid = Lid();
        lid.init();
        lid.setLockOpen(false);
        /* JSON init */
        json.init();
        /* Web Address */
        bbWebAddress.init();
        webAddress = bbWebAddress.getWebAddress();

        /* get reason of waking up, add task to queue */
        switch (esp_sleep_get_wakeup_cause())
        {
        /* timer was triggered, normal behaviour */
        case ESP_SLEEP_WAKEUP_TIMER:
        {
            ESP_LOGI("ESP_SLEEP", "weaken up from a timer");
            sim.setSleep(true);
            esp_wake_cycle++;

            ESP_LOGI("ESP_SLEEP", "cycle %i out of %i", esp_wake_cycle, ESP_SLEEP_CYCLES);
            if (esp_wake_cycle >= ESP_SLEEP_CYCLES)
            {
                int pin = TIMER_VIRTUAL_PIN;
                xQueueSend(gpio_evt_queue, &pin, NULL);
                esp_wake_cycle = 0;
            }
            break;
        }
        /* lid status was changed */
        case ESP_SLEEP_WAKEUP_EXT0:
        {
            ESP_LOGI("ESP_SLEEP", "lid status was changed");
            esp_wake_cycle = 0;

            int pin = GPIO_NUM_4;
            xQueueSend(gpio_evt_queue, &pin, NULL);
            break;
        }
        /* acceleration over max or user wants to wake up a box */
        case ESP_SLEEP_WAKEUP_EXT1:
        {
            ESP_LOGI("ESP_SLEEP", "acceleration over max or user wants to wake up a box");
            sim.setSleep(true);
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();

            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            printf("Wake up from a GPIO %d\n", pin);
            if (pin == MPU6050_INT_PIN)
            {
                mpuParser.update();
                esp_wake_cycle = 0;
                int pin2 = MPU6050_INT_PIN;
                xQueueSend(gpio_evt_queue, &pin2, NULL);
            }
            else if (pin == BUTTON_INT_PIN)
            {
                esp_wake_cycle = 0;
                int pin2 = BUTTON_INT_PIN;
                xQueueSend(gpio_evt_queue, &pin2, NULL);
            }
            /* unknown reason */
            else
            {
                ESP_LOGI("ESP_SLEEP", "unknown reason");
                sim.setSleep(true);
                esp_wake_cycle++;

                if (esp_wake_cycle >= ESP_SLEEP_CYCLES)
                {
                    int pin = TIMER_VIRTUAL_PIN;
                    xQueueSend(gpio_evt_queue, &pin, NULL);
                    esp_wake_cycle = 0;
                }
            }
            break;
        }
        /* clean reset, initialize devices */
        default:
        { // case ESP_SLEEP_WAKEUP_UNDEFINED: {
            ESP_LOGI("ESP_SLEEP", "clean reset, initialize devices");
            sim.setSleep(true);
            esp_wake_cycle = 0;

            int pin = TIMER_VIRTUAL_PIN;
            xQueueSend(gpio_evt_queue, &pin, NULL);
            vTaskDelay(200 / portTICK_PERIOD_MS);

            /* MPU6050 init */
            mpu.initialize();
            // mpu.setWakeCycleEnabled(true);
            /* Enable interrupts */
            mpu.setIntFreefallEnabled(true);
            mpu.setIntMotionEnabled(true);

            /* calibrate */
            mpu.CalibrateAccel(2 /*6*/); // TODO ??

            /* Set max motion */
            mpu.setFreefallDetectionThreshold(2);
            mpu.setFreefallDetectionDuration(1);
            mpu.setMotionDetectionThreshold(2);
            mpu.setMotionDetectionDuration(1);
            mpu.setMotionDetectionCounterDecrement(0);

            /* user console UART */
            BBCredentials bbCredentials = BBCredentials();
            bbCredentials.init();
            Console console(&bbCredentials, &bbWebAddress);
            console.init();
            if (console.enterConsole() == ConsoleOk)
                if (console.enterCredentials() == ConsoleOk)
                    console.enterConfig();

            break;
        }
            /* unknown reason */
            // default: {
            //     ESP_LOGI("ESP_SLEEP", "unknown reason");
            // }
        }
        // xTaskCreatePinnedToCore(sim_task, "sim_task", 12000, NULL, 10, &sim_task_handle, 0);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* start main */
        xTaskCreate(acc_task, "acc_task", 70000, NULL, 10, &acc_task_handle);
    }
#ifdef __cplusplus
}
#endif