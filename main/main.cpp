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

/* DEFINITIONS */
#define PIN_SDA 4
#define PIN_CLK 5

char webAddress[] = "ec2-3-237-238-240.compute-1.amazonaws.com/boxes/176:178:28:11:21:204";

/* HANDLERS */
TaskHandle_t sim_task_handle = NULL;
TaskHandle_t acc_task_handle = NULL;
TaskHandle_t send_task_handle = NULL;

/* SEMAPHORES */
SemaphoreHandle_t send_task_semaphore = NULL;
SemaphoreHandle_t message_update_semaphore = NULL;
SemaphoreHandle_t sim_task_semaphore = NULL;

/* MODULES INITIALIZATION */
JsonData json;
/* MPU6050 */
MPU6050 mpu = MPU6050();
MPU6050Parser mpuParser = MPU6050Parser(&mpu, 32768, 2);
/* SHT30 */
SHT30Esp8266 sht = SHT30Esp8266();
/* SIM800l */
Sim800lESP sim = Sim800lESP("google.com");
/* UART init */
uart_config_t uart_config_gps = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_DEFAULT};
/* NEO-6m */
GpsDownloaderESP downloader(&uart_config_gps);
GpsCmdsFinder finder(&downloader);
GpsGGA gga(&finder);
/* Lid */
Lid lid;

Message currentMessage(false, false, true);
ServerMessage pendingMessage(false, false, true, false);

/* FUNCTION'S DECLARATIONS */
static void reloadSimTask();
static void acc_task(void *arg);
static void sim_task(void *arg);

/* Reload sim task */
static void reloadSimTask() {
    sim_task_handle = NULL;
    xTaskCreate(sim_task, "sim_task", 12000, NULL, 10, &sim_task_handle);
    vTaskDelete(NULL);
}

static void update_message() {
    if(xSemaphoreTake(message_update_semaphore, portMAX_DELAY)) {
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

static void telemetry_send_task(void *arg) {
    if(xSemaphoreTake(send_task_semaphore, portMAX_DELAY)) {
    /* Make new JSON */
        char output[SIM800L_DEF_BUF_SIZE];
        memset(output, 0, SIM800L_DEF_BUF_SIZE);

        json.initNewMessage();

        json.addEspMac();
        json.addGpsInfo(gga.updateErr, (double)gga.getLatitude(), (double)gga.getLongitude());
        json.addAccInfo(0, (double)mpuParser.getAccX(), (double)mpuParser.getAccY(), (double)mpuParser.getAccZ());
        json.addTempInfo(/*shtErr*/0, (double)sht.getTemperature(), (double)sht.getHumidity());
        json.addTime(gga.getTime());
        json.addBatteryInfo(0, 100);
        char *jsonString = json.getJsonData();
        ESP_LOGI("JSON", "%s\n", jsonString);

        /* Send POST */
        /* Set AP *///TODO do better error handling
        ESP_LOGI("SIM", "Sending HTTP POST.");
        char address[100];
        memset(address, 0, 100);
        sprintf(address, "%s/telemetries", webAddress);
        Sim800lError err = sim.sendHTTPPOST(address, jsonString, output); //"{\n\"tt\":\t1\n}");
        if(err == Sim800lRecErr) {
            ESP_LOGI("SIM", "Sending HTTP POST failed.");
            //TODO save and send later
        } else if (err == Sim800lTimeoutErr || err == Sim800lErr) {
            ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
            sim.resetForce();
            // reloadSimTask();
            esp_restart();
        } else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr) {
            ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
            sim.reinit();
            // reloadSimTask();
            esp_restart();
        } else if (err == Sim800lOk)
            ESP_LOGI("SIM", "Sending HTTP POST OK.");
        
        json.deinitNewMessage();

        if(json.update(output) == JsonDataOk) {//TODO move to JsonData
            update_message();
        }

        xSemaphoreGive(send_task_semaphore);
        vTaskDelete(NULL);
    }

    
}

static void idle_get_task(void *arg) {
    if(xSemaphoreTake(send_task_semaphore, portMAX_DELAY)) {
        char output[SIM800L_DEF_BUF_SIZE];
        memset(output, 0, SIM800L_DEF_BUF_SIZE);

        /* Send GET */
        /* Set AP *///TODO do better error handling
        ESP_LOGI("SIM", "Sending HTTP GET.");
        char address[100];
        memset(address, 0, 100);
        sprintf(address, "%s/idle", webAddress);
        Sim800lError err = sim.sendHTTPGET(address, output); //"{\n\"tt\":\t1\n}");
        if(err == Sim800lRecErr) {
            ESP_LOGI("SIM", "Sending HTTP POST failed.");
            //TODO save and send later
        } else if (err == Sim800lTimeoutErr || err == Sim800lErr) {
            ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
            sim.resetForce();
            // reloadSimTask();
            esp_restart();
        } else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr) {
            ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
            sim.reinit();
            // reloadSimTask();
            esp_restart();
        } else if (err == Sim800lOk)
            ESP_LOGI("SIM", "Sending HTTP GET OK.");
        
        json.update(output);
        if(json.update(output) == JsonDataOk) {//TODO move to JsonData
            update_message();
        }

        xSemaphoreGive(send_task_semaphore);
        vTaskDelete(NULL);
    }

    
}

static void ans_send_task(void *arg) {
    if(xSemaphoreTake(send_task_semaphore, portMAX_DELAY)) {
    /* Make new JSON */
        char output[SIM800L_DEF_BUF_SIZE];
        memset(output, 0, SIM800L_DEF_BUF_SIZE);
        json.initNewMessage();

        json.addMessage(currentMessage);

        char *jsonString = json.getJsonData();
        ESP_LOGI("JSON", "%s\n", jsonString);

        /* Send POST */
        /* Set AP *///TODO do better error handling
        ESP_LOGI("SIM", "Sending HTTP POST.");
        char address[100];
        memset(address, 0, 100);
        sprintf(address, "%s/answer", webAddress);
        Sim800lError err = sim.sendHTTPPOST(address, jsonString, output); //"{\n\"tt\":\t1\n}");
        if(err == Sim800lRecErr) {
            ESP_LOGI("SIM", "Sending HTTP POST failed.");
            //TODO save and send later
        } else if (err == Sim800lTimeoutErr || err == Sim800lErr) {
            ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
            sim.resetForce();
            // reloadSimTask();
            esp_restart();
        } else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr) {
            ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
            sim.reinit();
            // reloadSimTask();
            esp_restart();
        } else if (err == Sim800lOk)
            ESP_LOGI("SIM", "Sending HTTP POST OK.");
        
        json.update(output);
        if(json.update(output) == JsonDataOk) {//TODO move to JsonData
            update_message();
        }

        json.deinitNewMessage();
    }

    xSemaphoreGive(send_task_semaphore);
    vTaskDelete(NULL);
}

/* Getting and transforming data && choosing correct messages and requests */
static void acc_task(void *arg)
{

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
    // not implemented
    /* NEO-6m init */
    GpsAllData gps = GpsAllData(&finder, &downloader);
    gps.add(&gga);
    ESP_LOGI("GPS:", "starting");
    downloader.init();
    /* Lid init */
    lid = Lid();
    lid.init();
    lid.setLockOpen(false);

    while (1)
    {
        /* Download a valid state from server */
        if(xSemaphoreTake(sim_task_semaphore, portMAX_DELAY)) {//add timer?
            if(currentMessage.getProtect() == true)
                xTaskCreate(telemetry_send_task, "telemetry_send_task", 12000, NULL, 10, &send_task_handle);
            else
                xTaskCreate(idle_get_task, "idle_get_task", 18000, NULL, 10, &send_task_handle);
        
            xSemaphoreGive(sim_task_semaphore);
        }
        /* MPU6050 */
        for(int i = 0; i < 7; i++) {
            mpuParser.update();
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
            if(xSemaphoreTake(message_update_semaphore, portMAX_DELAY) == pdTRUE) {
                ESP_LOGI("LID", "checking...");
                if(currentMessage.isSameWithAck(pendingMessage) == true && currentMessage.getAck() == true && pendingMessage.getValid() == true) {
                    /* Lid must be opened */
                    if(currentMessage.getOpen() == true) {
                        if(lid.getDetectorOpen() == true) {
                            lid.setLockOpen(false);
                        } else {
                            lid.setLockOpen(true);
                        }
                    } 
                    /* Lid must be closed */
                    else {
                        lid.setLockOpen(false);
                        if(lid.getDetectorOpen() == true) {
                            /* Send alarm to server *///TODO
                            
                        } else {
                            
                        }
                    }
                } 
                /* Something changed */
                else if(currentMessage.isSame(pendingMessage) == false) {
                    /* Lid lock change */
                    if(currentMessage.getOpen() != pendingMessage.getOpen()) {
                        /* Set open */
                        if(pendingMessage.getOpen() == true) {
                            currentMessage.setOpen(true);
                            if(lid.getDetectorOpen() == true) {
                                lid.setLockOpen(false);
                            } else {
                                lid.setLockOpen(true);
                            }
                        }  
                        /* Set close */
                        else {
                            lid.setLockOpen(false);
                            if(lid.getDetectorOpen() == false) {
                                currentMessage.setOpen(false);
                            } else {
                                /* wait */
                            }
                        }
                    }
                    /* Telemetry change */
                    else if(currentMessage.getProtect() != pendingMessage.getProtect()) {
                        if(pendingMessage.getProtect() == true)
                            currentMessage.setProtect(true);
                        else    
                            currentMessage.setProtect(false);
                    } 
                } 
                /* Everything else is ok so just send confirmation */
                if (currentMessage.isSame(pendingMessage) == true && pendingMessage.getAck() == false) {
                    currentMessage.setAck(true);
                    xTaskCreate(ans_send_task, "send_task", 12000, NULL, 10, &send_task_handle);
                }

                xSemaphoreGive(message_update_semaphore);
            }

            ESP_LOGI("WAIT", "");
            
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        
    }

    
}

static void sim_task(void *arg)
{
    if(xSemaphoreTake(sim_task_semaphore, portMAX_DELAY)) {
        /* SIM800l init*/
        if(sim.getInitStatus() != Sim800lUARTInitialised) {
            ESP_LOGI("SIM", "Starting initialization.");
            sim.init();
        }

        Sim800lError err = Sim800lErr;

        /* SIM800l synchronization */
        ESP_LOGI("SIM", "Starting synchronization.");
        err = sim.handshake(10);
        if(err == Sim800lErr || err == Sim800lTimeoutErr || err == Sim800lRecErr) {
            ESP_LOGI("SIM", "Proper response wasn't received %i times. Hardware SIM800l error. Rebooting device...", 10);
            esp_restart();
        } else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr) {
            ESP_LOGI("SIM", "UART perypherial error. Rebooting device...");
            esp_restart();
        } 
        ESP_LOGI("SIM", "Synchronization OK.");

        /* SIM card check */
        ESP_LOGI("SIM", "Checking SIM card.");
        err = sim.getSIMInfo();
        if(err == Sim800lRecErr) {
            ESP_LOGI("SIM", "Sim card not inserted, cannot progress. Rebooting device...");
            esp_restart();
        } else if (err == Sim800lTimeoutErr || err == Sim800lErr) {
            ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting device...");
            esp_restart();
        } else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr) {
            ESP_LOGI("SIM", "UART perypherial error. Rebooting device...");
            esp_restart();
        }
        ESP_LOGI("SIM", "SIM card OK.");

        /* Network check */
        // ESP_LOGI("SIM", "Checking if device registered to network.");
        // for(int i = 0; i < 5; i++) {
        //     err = sim.checkIfRegistered();
        //     if(err == Sim800lRegistered || err == Sim800lRoamingRegistered)
        //         break;
        //     else if(err == Sim800lRespErr && i < 4) {
        //         ESP_LOGI("SIM", "Cannot register, switching on and off airplane mode.");
        //         sim.airplaneModeEnable();
        //         vTaskDelay(100 / portTICK_PERIOD_MS);
        //         sim.airplaneModeDisable();
        //         vTaskDelay(2000 / portTICK_PERIOD_MS);
        //     }
        //     else if (err == Sim800lRespErr && i < 5) {
        //         ESP_LOGI("SIM", "Couldn't register to network %i times. Rebooting SIM800l...", 5);
        //         sim.resetForce();
        //         reloadSimTask();
        //     } else if (err == Sim800lTimeoutErr || err == Sim800lErr) {
        //         ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
        //         sim.resetForce();
        //         reloadSimTask();
        //     } else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr) {
        //         ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
        //         sim.reinit();
        //         reloadSimTask();
        //     }
        // }

        /* Set GPRS */
        ESP_LOGI("SIM", "Setting GPRS.");
        err = sim.setConnectionType("GPRS");
        if(err == Sim800lRecErr) {
            ESP_LOGI("SIM", "GPRS could not be set, cannot progress. Rebooting SIM800l...");
            sim.resetForce();
            reloadSimTask();
        } else if (err == Sim800lTimeoutErr || err == Sim800lErr) {
            ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
            sim.resetForce();
            reloadSimTask();
        } else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr) {
            ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
            sim.reinit();
            reloadSimTask();
        }
        ESP_LOGI("SIM", "Setting GPRS OK.");

        err = Sim800lErr;

        /* Set AP */
        ESP_LOGI("SIM:", "Setting AP to...");
        err = sim.setAccessPoint("Plus");
        if(err == Sim800lRecErr) {
            ESP_LOGI("SIM", "AP could not be set, cannot progress. Rebooting SIM800l...");
            sim.resetForce();
            reloadSimTask();
        } else if (err == Sim800lTimeoutErr || err == Sim800lErr) {
            ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
            sim.resetForce();
            reloadSimTask();
        } else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr) {
            ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
            sim.reinit();
            reloadSimTask();
        }
        ESP_LOGI("SIM", "Setting AP OK.");

        xSemaphoreGive(sim_task_semaphore);
    }

    vTaskDelete(sim_task_handle);
}

#ifdef __cplusplus
extern "C"
{
#endif
    void app_main(void)
    {
        /* INIT objects *///TODO
        send_task_semaphore = xSemaphoreCreateMutex();//xSemaphoreCreateBinary();
        message_update_semaphore = xSemaphoreCreateMutex();//xSemaphoreCreateBinary();
        sim_task_semaphore = xSemaphoreCreateMutex();
        // currentMessage = Message(false, false, true);
        // pendingMessage = Message(false, false, true);
        json = JsonData();
        json.init();
        
        // xTaskCreatePinnedToCore(sim_task, "sim_task", 12000, NULL, 10, &sim_task_handle, 0);
        xTaskCreate(sim_task, "sim_task", 12000, NULL, 10, &sim_task_handle);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        xTaskCreate(acc_task, "acc_task", 12000, NULL, 10, &acc_task_handle);
    }
#ifdef __cplusplus
}
#endif