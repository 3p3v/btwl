#include "Sim800lBB.hpp"
#include <cstring>
#include "driver/uart.h"
#include <esp_log.h>

Sim800lBB::Sim800lBB(const char * url) {
    memcpy(defaultUrl, url, strlen(url));
}

Sim800lBB::~Sim800lBB() {}

void Sim800lBB::setDefualtUrl(const char *url)
{
    memcpy(defaultUrl, url, strlen(url));
}

Sim800lError Sim800lBB::init()
{
    return Sim800lOk;
}

Sim800lError Sim800lBB::handshake()
{
    return execAT();
}

Sim800lError Sim800lBB::setConnectionType(const char *type)
{
    return writeSAPBR(3, 1, "CONTYPE", type);
}

Sim800lError Sim800lBB::setAccessPoint(const char *ap)
{
    return writeSAPBR(3, 1, "APN", ap);
}

Sim800lError Sim800lBB::sendHTTPPOST(const char *url, const char * data)
{
    writeSAPBR(1, 1);
    writeSAPBR(2, 1);
    execHTTPINIT();
    writeHTTPPARA("CID", 1);
    writeHTTPPARA("URL", url);
    writeHTTPPARA("CONTENT", "application/json");
    writeHTTPDATA(strlen(data), 15000, data);
    writeHTTPACTION(1);
    execHTTPREAD();
    //TODO
    char ans[1000] = {};
    int len = uart_read_bytes(UART_NUM_1, (unsigned char*)ans, 120 * 2, 10000 / portTICK_PERIOD_MS);
    ESP_LOGI("SIM","%i, %s", len, ans);
    //TODO
    execHTTPTERM();
    writeSAPBR(0, 1);

    return Sim800lOk;
}

Sim800lError Sim800lBB::sendHTTPPOST(const char * data)
{
    return sendHTTPPOST((const char *)defaultUrl, data);
}
