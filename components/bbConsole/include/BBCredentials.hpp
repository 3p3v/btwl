#pragma once

#include "nvs_flash.h"

#define CONSOLE_CREDENTIALS_LOG_SIZE 50
#define CONSOLE_CREDENTIALS_PASS_SIZE 50

typedef enum
{
    BBCredentialsOk = 0,
    BBCredentialsErr = 1
} BBCredentialsError;

class BBCredentials final
{
public:
    BBCredentials(){};
    ~BBCredentials(){};

    BBCredentialsError init()
    {
        bzero(log, CONSOLE_CREDENTIALS_LOG_SIZE);
        bzero(pass, CONSOLE_CREDENTIALS_PASS_SIZE);
        
        /* load login and password from nvs */
        esp_err_t err = nvs_open(nvcred, NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK)
        {
            ESP_LOGI("CONSOLE_CREDENTIALS", "Error (%s) passing NVS handle!\n", esp_err_to_name(err));
            esp_restart();
        }
        else
        {
            ESP_LOGI("CONSOLE_CREDENTIALS", "reading login");
            size_t nvsize = CONSOLE_CREDENTIALS_LOG_SIZE;
            err = nvs_get_str(nvs_handle, "log", log, &nvsize);
            switch (err)
            {
            case ESP_OK:
                ESP_LOGI("CONSOLE_CREDENTIALS", "Done\n");
                printf("%s\n", log);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
            {
                ESP_LOGI("CONSOLE_CREDENTIALS", "The value has not been not initialized yet!\n");
                strcpy(log, def_log);

                err = nvs_set_str(nvs_handle, "log", log);
                // ESP_LOGI("CONSOLE_CREDENTIALS", (err != ESP_OK) ? "Failed!\n" : "Done\n");

                ESP_LOGI("CONSOLE_CREDENTIALS", "saving");
                err = nvs_commit(nvs_handle);
                // ESP_LOGI("CONSOLE_CREDENTIALS", (err != ESP_OK) ? "Failed!\n" : "Done\n");
                break;
            }
            default:
                ESP_LOGI("CONSOLE_CREDENTIALS", "Error (%s) reading!\n", esp_err_to_name(err));
            }

            // printf("%s\n", pass);
            ESP_LOGI("CONSOLE_CREDENTIALS", "reading pass");
            // printf("\n");
            nvsize = CONSOLE_CREDENTIALS_PASS_SIZE;
            err = nvs_get_str(nvs_handle, "pass", pass, &nvsize);
            switch (err)
            {
            case ESP_OK:
                ESP_LOGI("CONSOLE_CREDENTIALS", "Done\n");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
            {
                ESP_LOGI("CONSOLE_CREDENTIALS", "The value has not been not initialized yet!\n");
                strcpy(pass, def_pass);

                err = nvs_set_str(nvs_handle, "pass", pass);
                // ESP_LOGI("CONSOLE_CREDENTIALS", (err != ESP_OK) ? "Failed!\n" : "Done\n");

                ESP_LOGI("CONSOLE_CREDENTIALS", "saving");
                err = nvs_commit(nvs_handle);
                // ESP_LOGI("CONSOLE_CREDENTIALS", (err != ESP_OK) ? "Failed!\n" : "Done\n");
                break;
            }
            default:
                ESP_LOGI("CONSOLE_CREDENTIALS", "Error (%s) reading!\n", esp_err_to_name(err));
            }

            nvs_close(nvs_handle);
        }

        
        printf("%s\n", pass);
        vTaskDelay(10);
        return BBCredentialsOk;
    }

    char *getLog()
    {
        // for(int i = 0; i < 50; i++)
        //     printf("%c\n", log[0]);
        return log;
    }

    BBCredentialsError setLog(char *newLog)
    {
        esp_err_t err = nvs_open(nvcred, NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK)
        {
            ESP_LOGI("CONSOLE_CREDENTIALS", "Error (%s) passing NVS handle!\n", esp_err_to_name(err));
            esp_restart();
        }
        else
        {
            ESP_LOGI("CONSOLE_CREDENTIALS", "setting login");
            strcpy(log, newLog);

            err = nvs_set_str(nvs_handle, "log", log);
            // ESP_LOGI("CONSOLE_CREDENTIALS", (err != ESP_OK) ? "Failed!\n" : "Done\n");

            ESP_LOGI("CONSOLE_CREDENTIALS", "saving");
            err = nvs_commit(nvs_handle);
            // ESP_LOGI("CONSOLE_CREDENTIALS", (err != ESP_OK) ? "Failed!\n" : "Done\n");
            nvs_close(nvs_handle);
        }

        return BBCredentialsOk;
    }

    char *getPass()
    {
        return pass;
    }

    BBCredentialsError setPass(char *newPass)
    {
        esp_err_t err = nvs_open(nvcred, NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK)
        {
            ESP_LOGI("CONSOLE_CREDENTIALS", "Error (%s) passing NVS handle!\n", esp_err_to_name(err));
            esp_restart();
        }
        else
        {
            ESP_LOGI("CONSOLE_CREDENTIALS", "setting password");
            strcpy(pass, newPass);

            err = nvs_set_str(nvs_handle, "pass", pass);
            // ESP_LOGI("CONSOLE_CREDENTIALS", (err != ESP_OK) ? "Failed!\n" : "Done\n");

            ESP_LOGI("CONSOLE_CREDENTIALS", "saving");
            err = nvs_commit(nvs_handle);
            // ESP_LOGI("CONSOLE_CREDENTIALS", (err != ESP_OK) ? "Failed!\n" : "Done\n");
            nvs_close(nvs_handle);
        }

        return BBCredentialsOk;
    }

    private:
        nvs_handle_t nvs_handle;
        char log[CONSOLE_CREDENTIALS_LOG_SIZE];
        char pass[CONSOLE_CREDENTIALS_PASS_SIZE];
        const char def_log[8] = "default";
        const char def_pass[8] = "default";
        const char nvcred[7] = "nvcred";
    };