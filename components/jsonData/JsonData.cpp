/*
 * JsonData.cpp
 *
 *  Created on: Apr 19, 2023
 *      Author: esp32
 */

#include <JsonData.hpp>
#include "freertos/FreeRTOS.h"
#include <esp_log.h>
#include "esp_mac.h"
#include <math.h>
#include <string.h>
// #include <esp_err.h>

JsonData::JsonData()
	: fullStats(nullptr), parsedData(nullptr), protect(false), open(false), ack(true) {}

JsonData::~JsonData() {
	// cJSON_Delete(fullStats);
}

float JsonData::roundUp(const double var, const int cut)
{
	float value = (int)(var * 100 + .5);
    return (float)value / 100;
}

JsonDataErr JsonData::check(cJSON *var)
{
    if(var == NULL) {
		return JsonDataAllErr;
	}

	return JsonDataOk;
}

JsonDataErr JsonData::init() {
//	cJSON_Hooks hooks;
//	hooks.free_fn = vPortFree;
//	hooks.malloc_fn = pvPortMalloc;
//	cJSON_InitHooks(&hooks);
	esp_efuse_mac_get_default(mac);
    // esp_read_mac(*mac_base, ESP_MAC_WIFI_STA);
	// esp_read_mac(mac, ESP_MAC_ETH);
	ESP_LOGI("JSON", "Got MAC");

	fullStats = cJSON_CreateObject();
	// ESP_LOGI("JSON", "Created full");

	if(fullStats == NULL) {
		// ESP_LOGI("JSON", "Deleting full");
		cJSON_Delete(fullStats);
		return JsonDataAllErr;
	}

	return JsonDataOk;
}

JsonDataErr JsonData::initNewMessage() {
	fullStats = cJSON_CreateObject();

	return JsonDataOk;
}

JsonDataErr JsonData::deinitNewMessage() {
	cJSON_Delete(fullStats);

	return JsonDataOk;
}

JsonDataErr JsonData::addEspMac() {
	char macStr[JSON_DATA_MAC_STR_LEN * 5] = {};
	snprintf(macStr, JSON_DATA_MAC_STR_LEN * 5, "%i:%i:%i:%i:%i:%i", (int)mac[0], (int)mac[1], (int)mac[2], (int)mac[3], (int)mac[4], (int)mac[5]);
	cJSON *id = cJSON_CreateString(macStr);

	if(id == NULL)
		return JsonDataAllErr;

	ESP_LOGI("JSON", "MAC added");
	cJSON_AddItemToObject(fullStats, "mac", id);

	return JsonDataOk;
}

//TODO
JsonDataErr JsonData::addBatteryInfo(int err, int level)
{
    cJSON_AddNumberToObject(fullStats, "batteryStatus", level);

	return JsonDataOk;
}

JsonDataErr JsonData::addTime(GpsTime time)
{
    int len = 0;
	
	char temp[JSON_DATA_TIME_LEN * 3] = {};
	if(time.hh < 10)
		len += snprintf(temp + len, 7, "%i", 0);
	len += snprintf(temp, 7, "%i", time.hh);
	temp[len] = ':';
	len++;
	if(time.mm < 10)
		len += snprintf(temp + len, 7, "%i", 0);
	len += snprintf(temp + len, 7, "%i", time.mm);
	temp[len] = ':';
	len++;
	if(time.ss < 10)
		len += snprintf(temp + len, 7, "%i", 0);
	len += snprintf(temp + len, 7, "%i", time.ss);

	cJSON_AddStringToObject(fullStats, "time", temp);

	return JsonDataOk;
}

JsonDataErr JsonData::addMessage(Message message)
{
    // cJSON *boolP = cJSON_CreateBool();
	cJSON_AddBoolToObject(fullStats, "protect", message.getProtect());
	cJSON_AddBoolToObject(fullStats, "open", message.getOpen());
	cJSON_AddBoolToObject(fullStats, "ack", message.getAck());

	return JsonDataOk;
}

JsonDataErr JsonData::addAlarm(int code)
{
    cJSON_AddNumberToObject(fullStats, "code", code);
	return JsonDataOk;
}

char *JsonData::getJsonData()
{
    // ESP_LOGI("JSON", "JSON parsed");
	parsedData = cJSON_Print(fullStats);
	return parsedData;
}

JsonDataErr JsonData::addGpsInfo(int err, GpsFloat lati, GpsFloat longi) {
	// char latitude[JSON_DATA_LATITUDE_LEN + 1];
	// char longitude[JSON_DATA_LONGITUDE_LEN + 1];
	// BBFloat bb = BBFloat();

	/* Changing ints to c strings */
	// bb.floatToBBFloat(lati, MPU6050_DIGITS);
	// bb.bbFloatToString(latitude, JSON_DATA_LATITUDE_SING_LEN, JSON_DATA_LATITUDE_NSING_LEN);
	// cJSON *lat = cJSON_CreateString(latitude);

	// bb.floatToBBFloat(longi, MPU6050_DIGITS);
	// bb.bbFloatToString(longitude, JSON_DATA_LONGITUDE_SING_LEN, JSON_DATA_LONGITUDE_NSING_LEN);
	// cJSON *lon = cJSON_CreateString(longitude);

	// if(lat == NULL || lon == NULL)
	// 	return JsonDataAllErr;

	cJSON_AddNumberToObject(fullStats, "latitude", lati);
	cJSON_AddNumberToObject(fullStats, "longitude", longi);
	return JsonDataOk;
}

JsonDataErr JsonData::addAccInfo(int err, MPU6050Float x, MPU6050Float y, MPU6050Float z) {
	// char xyz[JSON_DATA_ACC_LEN + 2];
	// BBFloat bb = BBFloat();

	// bb.floatToBBFloat(x, MPU6050_DIGITS);
	// bb.bbFloatToString(xyz);
	// cJSON * accX = cJSON_CreateString(xyz);

	// if(accX == NULL)
	// 	return JsonDataAllErr;

	// bb.floatToBBFloat(y, MPU6050_DIGITS);
	// bb.bbFloatToString(xyz);
	// cJSON * accY = cJSON_CreateString(xyz);

	// if(accY == NULL)
	// 	return JsonDataAllErr;

	// bb.floatToBBFloat(z, MPU6050_DIGITS);
	// bb.bbFloatToString(xyz);
	// cJSON * accZ = cJSON_CreateString(xyz);

	// if(accZ == NULL)
	// 	return JsonDataAllErr;

	// cJSON_AddItemToObject(fullStats, "accX", accX);
	// cJSON_AddItemToObject(fullStats, "accZ", accZ);
	MPU6050Float ret = x;

	if(y > ret)
		ret = y;

	if(z > ret)
		ret = z;

	cJSON_AddNumberToObject(fullStats, "maxAcceleration", ret);

	return JsonDataOk;
}

JsonDataErr JsonData::addTempInfo(int err, SHT30Float temperature, SHT30Float humidity) {
	// char data[JSON_DATA_SHT_LEN + 2] = {};
	// BBFloat bb = BBFloat();

	/* Changing ints to c strings */
	// bb.floatToBBFloat(temperature, SHT30_TEMP_NSIGN_POINT);
	// bb.bbFloatToString(data);
	// cJSON * temp = cJSON_CreateString(data);

	// if(temp == NULL)
	// 	return JsonDataAllErr;

	// bb.floatToBBFloat(humidity, SHT30_TEMP_NSIGN_POINT);
	// bb.bbFloatToString(data);
	// cJSON * hum = cJSON_CreateString(data);

	// if(hum == NULL)
	// 	return JsonDataAllErr;

	cJSON_AddNumberToObject(fullStats, "temperature", temperature);
	cJSON_AddNumberToObject(fullStats, "humidity", humidity);

	return JsonDataOk;
}

JsonDataErr JsonData::update(char * raw) {
	char *start = strchr(raw, (int)'{');
	char *end = strchr(raw, (int)'}');

	ESP_LOGI("JSON", "check -0");

	if(start == NULL || end == NULL)
		return JsonDataError;

	char newJ[100];
	memset(newJ, '0', 100);
	int i = 0;
	while(1) {
		if((start - 1) != end) {
			newJ[i] = *start;
			start++;
			i++;
		} else {
			break;
		}
	}

	const cJSON *jsonProtect = NULL;
	const cJSON *jsonOpen = NULL;
	const cJSON *jsonAck = NULL;
	cJSON *monitor_json = cJSON_Parse(newJ);
	ESP_LOGI("JSON", "check 0");

	if (monitor_json == NULL) {
		ESP_LOGI("JSON", "check 0");
		return JsonDataError;
	}

	jsonProtect = cJSON_GetObjectItemCaseSensitive(monitor_json, "protect");
	ESP_LOGI("JSON", "check 1");
	if(cJSON_IsBool(jsonProtect) == true) {
		if(cJSON_IsTrue(jsonProtect)) {
			protect = true;
		} else {
			protect = false;
		}
	} else {
		ESP_LOGI("JSON", "check 1");
		return JsonDataError;
	}

	jsonOpen = cJSON_GetObjectItemCaseSensitive(monitor_json, "open");
	ESP_LOGI("JSON", "check 2");
	if(cJSON_IsBool(jsonOpen) == true) {
		if(cJSON_IsTrue(jsonOpen)) {
			open = true;
		} else {
			open = false;
		}
	} else {
		ESP_LOGI("JSON", "check 2");
		return JsonDataError;
	}

	jsonAck = cJSON_GetObjectItemCaseSensitive(monitor_json, "ack");
	ESP_LOGI("JSON", "check 3");
	if(cJSON_IsBool(jsonAck) == true) {
		if(cJSON_IsTrue(jsonAck)) {
			ack = true;
		} else {
			ack = false;
		}
	} else {
		ESP_LOGI("JSON", "check 3");
		return JsonDataError;
	}

	cJSON_Delete(monitor_json);

	return JsonDataOk;
}

bool JsonData::getProtect() {
	return protect;
}

bool JsonData::getOpen() {
	return open;
}

bool JsonData::getAck() {
	return ack;
}
