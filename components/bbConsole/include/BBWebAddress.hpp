#pragma once

#include "esp_mac.h"
#include "nvs_flash.h"

#define WEB_ADDRESS_MAC_SIZE 6
#define WEB_ADDRESS_MAC_STR_SIZE 17 * 5
#define WEB_ADDRESS_SERVER_IP_SIZE 50
#define WEB_ADDRESS_WEB_ADDRESS_SIZE 150

typedef enum
{
    BBWebAddressOk = 0,
    BBWebAddressErr = 1
} BBWebAddressError;

class BBWebAddress
{
public:
    BBWebAddress(){};
    ~BBWebAddress(){};

    BBWebAddressError init()
    {
        /* init vars */
        bzero(server_ip, WEB_ADDRESS_SERVER_IP_SIZE);
        bzero(webAddress, WEB_ADDRESS_WEB_ADDRESS_SIZE);

        /* read MAC */
        ESP_LOGI("CONSOLE_WEB_ADDRESS", "reading MAC\n");
        bzero(mac_str, WEB_ADDRESS_MAC_STR_SIZE);
        unsigned char mac[WEB_ADDRESS_MAC_SIZE];
        esp_efuse_mac_get_default(mac);
        snprintf(mac_str, WEB_ADDRESS_MAC_STR_SIZE, "%i:%i:%i:%i:%i:%i", (int)mac[0], (int)mac[1], (int)mac[2], (int)mac[3], (int)mac[4], (int)mac[5]);

        /* load server ip from nvs */
        esp_err_t err = nvs_open(nvadd, NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK)
        {
            ESP_LOGI("CONSOLE_WEB_ADDRESS", "Error (%s) passing NVS handle!\n", esp_err_to_name(err));
            esp_restart();
        }
        else
        {
            ESP_LOGI("CONSOLE_WEB_ADDRESS", "reading server_ip");
            size_t nvsize = WEB_ADDRESS_SERVER_IP_SIZE;
            err = nvs_get_str(nvs_handle, "server_ip", server_ip, &nvsize);
            switch (err)
            {
            case ESP_OK:
                ESP_LOGI("CONSOLE_WEB_ADDRESS", "Done\n");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
            {
                ESP_LOGI("CONSOLE_WEB_ADDRESS", "server ip has not been not initialized yet!\n");
                strcpy(server_ip, def_server_ip);

                err = nvs_set_str(nvs_handle, "server_ip", server_ip);
                // ESP_LOGI("CONSOLE_WEB_ADDRESS", (err != ESP_OK) ? "Failed!\n" : "Done\n");

                ESP_LOGI("CONSOLE_WEB_ADDRESS", "saving");
                err = nvs_commit(nvs_handle);
                // ESP_LOGI("CONSOLE_WEB_ADDRESS", (err != ESP_OK) ? "Failed!\n" : "Done\n");
                break;
            }
            default:
                ESP_LOGI("CONSOLE_WEB_ADDRESS", "Error (%s) reading!\n", esp_err_to_name(err));
            }

            nvs_close(nvs_handle);
        }

        /* build a full address */
        set_full_address();

        return BBWebAddressOk;
    }

    char *getWebAddress()
    {
        return webAddress;
    }

    char *getServerIP()
    {
        return server_ip;
    }

    BBWebAddressError setServerIP(char *new_server_ip)
    {
        /* init vars */
        bzero(server_ip, WEB_ADDRESS_SERVER_IP_SIZE);
        bzero(webAddress, WEB_ADDRESS_WEB_ADDRESS_SIZE);

        esp_err_t err = nvs_open(nvadd, NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK)
        {
            ESP_LOGI("CONSOLE_WEB_ADDRESS", "Error (%s) passing NVS handle!\n", esp_err_to_name(err));
            esp_restart();
        }
        else
        {
            ESP_LOGI("CONSOLE_WEB_ADDRESS", "setting server ip");
            strcpy(server_ip, def_server_ip);

            err = nvs_set_str(nvs_handle, "server_ip", server_ip);
            // ESP_LOGI("CONSOLE_WEB_ADDRESS", (err != ESP_OK) ? "Failed!\n" : "Done\n");

            ESP_LOGI("CONSOLE_WEB_ADDRESS", "saving");
            err = nvs_commit(nvs_handle);
            // ESP_LOGI("CONSOLE_WEB_ADDRESS", (err != ESP_OK) ? "Failed!\n" : "Done\n");
            nvs_close(nvs_handle);
        }

        /* build a full address */
        set_full_address();

        return BBWebAddressOk;
    }

private:
    nvs_handle_t nvs_handle;

    
    // const char boxes_uri[] = "/boxes/";
    char mac_str[WEB_ADDRESS_MAC_STR_SIZE];

    char server_ip[WEB_ADDRESS_SERVER_IP_SIZE];
    char webAddress[WEB_ADDRESS_WEB_ADDRESS_SIZE];

    const char nvadd[6] = "nvadd";
    const char def_server_ip[32] = "3.215.18.200";

    void set_full_address()
    {
        snprintf(webAddress, WEB_ADDRESS_WEB_ADDRESS_SIZE, "%s/boxes/%s", server_ip, mac_str);
    }
};