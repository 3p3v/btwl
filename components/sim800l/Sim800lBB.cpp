#include "Sim800lBB.hpp"
#include <cstring>
#include "driver/uart.h"
#include <esp_log.h>

Sim800lBB::Sim800lBB(const char * url)
    :conFlags(0), sleep(true) {
    memcpy(defaultUrl, url, strlen(url));
}

Sim800lBB::~Sim800lBB() {}

bool Sim800lBB::getSleep()
{
    return sleep;
}

void Sim800lBB::setSleep(bool sleep)
{
    this->sleep = sleep;
}

void Sim800lBB::setDefualtUrl(const char *url)
{
    memset(defaultUrl, 0, sizeof(defaultUrl));
    memcpy(defaultUrl, url, strlen(url));
}

Sim800lError Sim800lBB::init()
{
    return Sim800lOk;
}

Sim800lError Sim800lBB::deinit()
{
    return Sim800lError();
}

Sim800lError Sim800lBB::reinit()
{
    return Sim800lError();
}

Sim800lError Sim800lBB::commonSettings()
{
    Sim800lError err = Sim800lErr;

        /* SIM800l synchronization */
        ESP_LOGI("SIM", "Starting synchronization.");
        err = handshake(10);
        if (err == Sim800lErr || err == Sim800lTimeoutErr || err == Sim800lRecErr)
        {
            ESP_LOGI("SIM", "Proper response wasn't received %i times. Hardware SIM800l error. Rebooting device...", 10);
            esp_restart();
        }
        else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr)
        {
            ESP_LOGI("SIM", "UART perypherial error. Rebooting device...");
            esp_restart();
        }
        ESP_LOGI("SIM", "Synchronization OK.");

        /* SIM card check */
        ESP_LOGI("SIM", "Checking SIM card.");
        err = getSIMInfo();
        if (err == Sim800lRecErr)
        {
            ESP_LOGI("SIM", "Sim card not inserted, cannot progress. Rebooting device...");
            esp_restart();
        }
        else if (err == Sim800lTimeoutErr || err == Sim800lErr)
        {
            ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting device...");
            esp_restart();
        }
        else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr)
        {
            ESP_LOGI("SIM", "UART perypherial error. Rebooting device...");
            esp_restart();
        }
        ESP_LOGI("SIM", "SIM card OK.");

        /* Network check */
        // ESP_LOGI("SIM", "Checking if device registered to network.");
        // for(int i = 0; i < 5; i++) {
        //     err = checkIfRegistered();
        //     if(err == Sim800lRegistered || err == Sim800lRoamingRegistered)
        //         break;
        //     else if(err == Sim800lRespErr && i < 4) {
        //         ESP_LOGI("SIM", "Cannot register, switching on and off airplane mode.");
        //         airplaneModeEnable();
        //         vTaskDelay(100 / portTICK_PERIOD_MS);
        //         airplaneModeDisable();
        //         vTaskDelay(2000 / portTICK_PERIOD_MS);
        //     }
        //     else if (err == Sim800lRespErr && i < 5) {
        //         ESP_LOGI("SIM", "Couldn't register to network %i times. Rebooting SIM800l...", 5);
        //         resetForce();
        //         reloadSimTask();
        //     } else if (err == Sim800lTimeoutErr || err == Sim800lErr) {
        //         ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
        //         resetForce();
        //         reloadSimTask();
        //     } else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr) {
        //         ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
        //         reinit();
        //         reloadSimTask();
        //     }
        // }

        /* Set GPRS */
        ESP_LOGI("SIM", "Setting GPRS.");
        err = setConnectionType("GPRS");
        if (err == Sim800lRecErr)
        {
            ESP_LOGI("SIM", "GPRS could not be set, cannot progress. Rebooting SIM800l...");
            resetForce();
            esp_restart();
            // reloadSimTask();
        }
        else if (err == Sim800lTimeoutErr || err == Sim800lErr)
        {
            ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
            resetForce();
            esp_restart();
            // reloadSimTask();
        }
        else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr)
        {
            ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
            reinit();
            esp_restart();
            // reloadSimTask();
        }
        ESP_LOGI("SIM", "Setting GPRS OK.");

        err = Sim800lErr;

        /* Set AP */
        ESP_LOGI("SIM:", "Setting AP to...");
        err = setAccessPoint("Plus");
        if (err == Sim800lRecErr)
        {
            ESP_LOGI("SIM", "AP could not be set, cannot progress. Rebooting SIM800l...");
            resetForce();
            esp_restart();
            // reloadSimTask();
        }
        else if (err == Sim800lTimeoutErr || err == Sim800lErr)
        {
            ESP_LOGI("SIM", "SIM800l not responding correctly. Hardware SIM800l error. Rebooting SIM800l...");
            resetForce();
            esp_restart();
            // reloadSimTask();
        }
        else if (err == Sim800lHardwareErr || err == Sim800lBufferFullErr)
        {
            ESP_LOGI("SIM", "UART perypherial error. Restarting UART...");
            reinit();
            esp_restart();
            // reloadSimTask();
        }
        ESP_LOGI("SIM", "Setting AP OK.");

        return err;
        // err = writeSAPBR(1, 1);
        // if(err != Sim800lOk) {
        //     resetForce();
        //     reloadSimTask();
        // }

        // err = writeSAPBR(2, 1);
        // if(err != Sim800lOk) {
        //     resetForce();
        //     reloadSimTask();
        // }
}

Sim800lError Sim800lBB::handshake()
{
    return execAT();
}

Sim800lError Sim800lBB::handshake(int x)
{
    for(int i = 0; i < x; i++){
        if(execAT() == Sim800lOk)
            return Sim800lOk;

        simDelay(2000);
    }

    return Sim800lErr;
}

Sim800lError Sim800lBB::getSIMInfo()
{
    return execCCID();
}

Sim800lError Sim800lBB::checkIfRegistered()
{
    Sim800lError err = readCREG();
    if(err != Sim800lOk)
        return err;

    char sub[] = "+CREG:";
    char * ptr = strstr(receivedData, sub);

    if(ptr == NULL)
        return Sim800lErr;

    ptr += strlen(sub) + 4;

    if(*ptr == '1')
        return Sim800lRegistered;
    else if(*ptr == '5')
        return Sim800lRoamingRegistered;
    else 
        return Sim800lRespErr;
}

Sim800lError Sim800lBB::setConnectionType(const char *type)
{
    return writeSAPBR(3, 1, "CONTYPE", type);
}

Sim800lError Sim800lBB::setAccessPoint(const char *ap)
{
    return writeSAPBR(3, 1, "APN", ap);
}

Sim800lError Sim800lBB::sendHTTPPOST(const char *url, const char * data, char * output)
{
    // Sim800lError err;
    Sim800lError err = writeSAPBR(1, 1);
    // // if(err != Sim800lOk) 
    // //     return err;

    err = writeSAPBR(2, 1);
    // if(err != Sim800lOk)
    //     goto CON_DEINIT;

    err = execHTTPINIT();
    // if(err != Sim800lOk) 
    //     goto HTTP_DEINIT;

    err = writeHTTPPARA("CID", 1);
    // if(err != Sim800lOk)
    //     goto HTTP_DEINIT;

    err = writeHTTPPARA("URL", url);
    // if(err != Sim800lOk)
    //     goto HTTP_DEINIT;

    err = writeHTTPPARA("CONTENT", "application/json");
    // if(err != Sim800lOk)
    //     goto HTTP_DEINIT;

    err = writeHTTPDATA(strlen(data), 15000, data);
    // if(err != Sim800lOk)
    //     goto HTTP_DEINIT;

    err = writeHTTPACTION(1);
    // if(err != Sim800lOk)
    //     goto HTTP_DEINIT;

    err = execHTTPREAD();
    if(err != Sim800lOk)
        goto HTTP_DEINIT;

    strcpy(output, receivedData);
    // memcpy(output, receivedData, receivedLen);
    ESP_LOGI("HTTP", "message: %s", output);

    err = execHTTPTERM();
    if(err != Sim800lOk)
        goto HTTP_DEINIT;

    err = writeSAPBR(0, 1);
    if(err != Sim800lOk)
        goto CON_DEINIT;

    return Sim800lOk;

    CON_DEINIT:
    writeSAPBR(0, 1);
    return err;

    HTTP_DEINIT:
    writeSAPBR(0, 1);
    execHTTPTERM();
    return err;
}

Sim800lError Sim800lBB::sendHTTPPOST(const char * data, char * output)
{
    return sendHTTPPOST((const char *)defaultUrl, data, output);
}

Sim800lError Sim800lBB::sendHTTPGET(const char *url, char * output)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Sim800lError err = writeSAPBR(1, 1);
    err = writeSAPBR(1, 1);
    err = writeSAPBR(1, 1);
    // if(err != Sim800lOk) 
    //     return err;

    err = writeSAPBR(2, 1);
    err = writeSAPBR(2, 1);
    err = writeSAPBR(2, 1);
    // if(err != Sim800lOk)
    //     goto CON_DEINIT;

    err = execHTTPINIT();
    // if(err != Sim800lOk) 
    //     goto HTTP_DEINIT;

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    err = writeHTTPPARA("CID", 1);
    // if(err != Sim800lOk)
    //     goto HTTP_DEINIT;

    err = writeHTTPPARA("URL", url);
    // if(err != Sim800lOk)
    //     goto HTTP_DEINIT;

    err = writeHTTPACTION(0);
    // if(err != Sim800lOk)
    //     goto HTTP_DEINIT;

    err = execHTTPREAD();
    if(err != Sim800lOk)
        goto HTTP_DEINIT;

    strcpy(output, receivedData);
    // memcpy(output, receivedData, receivedLen);
    ESP_LOGI("HTTP", "message: %s", output);

    err = execHTTPTERM();
    if(err != Sim800lOk)
        goto HTTP_DEINIT;

    err = writeSAPBR(0, 1);
    if(err != Sim800lOk)
        goto CON_DEINIT;

    return Sim800lOk;

    CON_DEINIT:
    writeSAPBR(0, 1);
    return err;

    HTTP_DEINIT:
    writeSAPBR(0, 1);
    execHTTPTERM();
    return err;
}

Sim800lError Sim800lBB::sleepModeEnable()
{
    Sim800lError err = Sim800lErr;
    
    for(int i = 0; i < 10; i++) {
        if(setDRT(Sim800lPinHigh) == Sim800lOk)
            err = writeCSCLK(1);

        setDRT(Sim800lPinLow); 

        if(err == Sim800lOk) {
            sleep = true;
            return err;
        }
    }        

    return Sim800lErr;
}

Sim800lError Sim800lBB::sleepModeDisable()
{
    Sim800lError err = Sim800lErr;
    
    for(int i = 0; i < 10; i++) {
        if(setDRT(Sim800lPinHigh) == Sim800lOk)
            err = writeCSCLK(0);

        setDRT(Sim800lPinLow); 

        if(err == Sim800lOk){
            sleep = false;
            return err;
        }
    }        

    return Sim800lErr;
}

Sim800lError Sim800lBB::airplaneModeEnable()
{
    return writeCFUN(4);
}

Sim800lError Sim800lBB::airplaneModeDisable()
{
    return writeCFUN(1);
}

Sim800lError Sim800lBB::resetNormal()
{
    return writeCPOWD(1);
}

Sim800lError Sim800lBB::resetUrgent()
{
    return writeCPOWD(0);
}

//TODO
Sim800lError Sim800lBB::resetForce()
{
    if(setRST(Sim800lPinLow) == Sim800lOk)
        return setRST(Sim800lPinHigh);
    else
        return Sim800lErr;
}

void Sim800lBB::setStatus(const Sim800lStatus status)
{
    conFlags |= (0x1 << status);
}

void Sim800lBB::resetStatus(const Sim800lStatus status)
{
    conFlags &= ~(0x1 << status);
}

Sim800lStatus Sim800lBB::getGPRSStatus() {
    // return (Sim800lStatus)(conFlags & (uint8_t)Sim800lGPRSConnected);
    return (Sim800lStatus)0;
}

Sim800lStatus Sim800lBB::getHTTPStatus() {
    // return (Sim800lStatus)(conFlags & (uint8_t)Sim800lHTTPInitialised);
    return (Sim800lStatus)0;
}

Sim800lStatus Sim800lBB::getInitStatus()
{
    return (Sim800lStatus)(conFlags & (uint8_t)Sim800lUARTInitialised);
}
