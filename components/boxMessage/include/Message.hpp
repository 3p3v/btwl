#pragma once

#include "nvs_flash.h"
#include "esp_log.h"

class Message {
public:
    Message(bool protect, bool open, bool ack)
        : protect(protect), open(open), ack(ack) {}

    bool getProtect() {
        return protect;
    }
	bool getOpen() {
        return open;
    }
	bool getAck() {
        return ack;
    }
    virtual void setProtect(bool newV) {
        protect = newV;
    }
	virtual void setOpen(bool newV) {
        open = newV;
    }
	void setAck(bool newV) {
        ack = newV;
    }

    bool isSame(Message message) {
        if(message.getProtect() == protect && message.getOpen() == open)
            return true;
        else
            return false;
    }

    bool isSameWithAck(Message message) {
        if(message.getProtect() == protect && message.getOpen() == open && message.getAck() == ack)
            return true;
        else 
            return false;
    }

protected:
    bool protect;
	bool open;
	bool ack;
};

class ServerMessage final : public Message {
public:
    ServerMessage(bool protect, bool open, bool ack, bool valid)
        : Message(protect, open, ack), valid(valid) {}

    void setValid(bool valid) {
        this->valid = valid;
    }

    bool getValid() {
        return valid;
    }

private:
    bool valid;
};

class ESPNonVolatileMessage final : public Message {
public:
    ESPNonVolatileMessage(bool protect, bool open, bool ack)
        : Message(protect, open, ack) {};

    ESPNonVolatileMessage() 
        : Message(false, false, true) {};

    ~ESPNonVolatileMessage() {};

    void init() {
        esp_err_t err = nvs_open("nvmessage", NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGI("ESPNonVolatileMessage", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            esp_restart();
        } else {
            ESP_LOGI("ESPNonVolatileMessage", "reading protect");
            uint8_t temp = 0;
            err = nvs_get_u8(nvs_handle, "protect", &temp);
            switch (err) {
                case ESP_OK:
                    ESP_LOGI("ESPNonVolatileMessage", "Done\n");
                    protect = (bool)temp;
                    break;
                case ESP_ERR_NVS_NOT_FOUND: {
                    ESP_LOGI("ESPNonVolatileMessage", "The value has not been not initialized yet!\n");

                    ESP_LOGI("ESPNonVolatileMessage", "saving");
                    err = nvs_set_u8(nvs_handle, "protect", (uint8_t)false);
                    // ESP_LOGI("ESPNonVolatileMessage", (err != ESP_OK) ? "Failed!\n" : "Done\n");

                    ESP_LOGI("ESPNonVolatileMessage", "saving");
                    err = nvs_commit(nvs_handle);
                    // ESP_LOGI("ESPNonVolatileMessage", (err != ESP_OK) ? "Failed!\n" : "Done\n");
                    break;
                }
                default :
                    ESP_LOGI("ESPNonVolatileMessage", "Error (%s) reading!\n", esp_err_to_name(err));
            }

            ESP_LOGI("ESPNonVolatileMessage", "reading open");
            temp = 0;
            err = nvs_get_u8(nvs_handle, "open", &temp);
            switch (err) {
                case ESP_OK:
                    open = (bool)temp;
                    ESP_LOGI("ESPNonVolatileMessage", "Done\n");
                    break;
                case ESP_ERR_NVS_NOT_FOUND: {
                    ESP_LOGI("ESPNonVolatileMessage", "The value has not been not initialized yet!\n");

                    ESP_LOGI("ESPNonVolatileMessage", "saving");
                    err = nvs_set_u8(nvs_handle, "open", (uint8_t)false);
                    // ESP_LOGI("ESPNonVolatileMessage", (err != ESP_OK) ? "Failed!\n" : "Done\n");

                    ESP_LOGI("ESPNonVolatileMessage", "saving");
                    err = nvs_commit(nvs_handle);
                    // ESP_LOGI("ESPNonVolatileMessage", (err != ESP_OK) ? "Failed!\n" : "Done\n");
                    break;
                }
                default :
                    ESP_LOGI("ESPNonVolatileMessage", "Error (%s) reading!\n", esp_err_to_name(err));
            }

            nvs_close(nvs_handle);
        }
    }

    virtual void setProtect(bool newV) override {
        protect = newV;

        esp_err_t err = nvs_open("nvmessage", NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGI("ESPNonVolatileMessage", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            esp_restart();
        } else {
            ESP_LOGI("ESPNonVolatileMessage", "saving");
            err = nvs_set_u8(nvs_handle, "protect", (uint8_t)newV);
            // ESP_LOGI("ESPNonVolatileMessage", (err != ESP_OK) ? "Failed!\n" : "Done\n");

            ESP_LOGI("ESPNonVolatileMessage", "saving");
            err = nvs_commit(nvs_handle);
            // ESP_LOGI("ESPNonVolatileMessage", (err != ESP_OK) ? "Failed!\n" : "Done\n");

            nvs_close(nvs_handle);
        }   
    }
	virtual void setOpen(bool newV) override {
        open = newV;
        esp_err_t err = nvs_open("nvmessage", NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGI("ESPNonVolatileMessage", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            esp_restart();
        } else {
            ESP_LOGI("ESPNonVolatileMessage", "saving");
            err = nvs_set_u8(nvs_handle, "open", (uint8_t)newV);
            // ESP_LOGI("ESPNonVolatileMessage", (err != ESP_OK) ? "Failed!\n" : "Done\n");

            ESP_LOGI("ESPNonVolatileMessage", "saving");
            err = nvs_commit(nvs_handle);
            // ESP_LOGI("ESPNonVolatileMessage", (err != ESP_OK) ? "Failed!\n" : "Done\n");

            nvs_close(nvs_handle);
        }
    }

private:
    nvs_handle_t nvs_handle;
};