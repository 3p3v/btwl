/*
 * DataParser.hpp
 *
 *  Created on: Apr 19, 2023
 *      Author: esp32
 */

#pragma once

#include "cJSON.h"
#include "gps.hpp"
#include "MPU6050Parser.hpp"
#include "SHT30.hpp"

#define JSON_DATA_MAC_LEN				6
#define JSON_DATA_MAC_STR_LEN			17

#define JSON_DATA_POINT_LEN				1

#define JSON_DATA_TIME_LEN				9

#define JSON_DATA_LATITUDE_SING_LEN		2
#define JSON_DATA_LONGITUDE_SING_LEN	3
#define JSON_DATA_LATITUDE_NSING_LEN	7
#define JSON_DATA_LONGITUDE_NSING_LEN	7
#define JSON_DATA_LATITUDE_LEN			JSON_DATA_LATITUDE_SING_LEN + JSON_DATA_LATITUDE_NSING_LEN + JSON_DATA_POINT_LEN
#define JSON_DATA_LONGITUDE_LEN			JSON_DATA_LONGITUDE_SING_LEN + JSON_DATA_LONGITUDE_NSING_LEN + JSON_DATA_POINT_LEN

#define JSON_DATA_ACC_SING_LEN			2
#define JSON_DATA_ACC_NSING_LEN			4
#define JSON_DATA_ACC_LEN				JSON_DATA_ACC_SING_LEN + JSON_DATA_ACC_NSING_LEN + JSON_DATA_POINT_LEN

#define JSON_DATA_SHT_SIGN_LEN			3
#define JSON_DATA_SHT_SIGN_LEN			SHT30_TEMP_NSIGN_POINT
#define JSON_DATA_SHT_LEN				JSON_DATA_SHT_SIGN_LEN + JSON_DATA_SHT_SIGN_LEN + JSON_DATA_POINT_LEN

typedef enum {
	JsonDataOk = 0,
	JsonDataAllErr = 1
} JsonDataErr;

class JsonData final{
public:
	JsonData();
	virtual ~JsonData();

	JsonDataErr init();

	/* Add ESP Id */
	JsonDataErr addEspMac();

	/* Add GPS info */
	JsonDataErr addGpsInfo(int err, GpsFloat latitude, GpsFloat longitude);
	JsonDataErr addGpsInfo(int err);

	/* Add accelerometer info */
	JsonDataErr addAccInfo(int err, MPU6050Float X, MPU6050Float Y, MPU6050Float Z);

	/* Add temperature info */
	JsonDataErr addTempInfo(int err, SHT30Float temperature, SHT30Float humidity);

	/* Add door info */
	JsonDataErr addDoorInfo(int err);

	/* Add battery level percent info */
	JsonDataErr addBatteryInfo(int err, int level);

	JsonDataErr addTime(GpsTime time);

	/* Parse and return a pointer to data */
	char* getJsonData();

private:
	cJSON * fullStats;
	char * parsedData;
	unsigned char mac[JSON_DATA_MAC_LEN];

	JsonDataErr check(cJSON * var);
};

