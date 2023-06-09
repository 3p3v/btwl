/*
 * gps_data.hpp
 *
 *  Created on: Apr 4, 2023
 *      Author: esp32
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include <cmath>
#include "minmea.h"
#include "BBFormats.hpp"

#define MINMEA_MAX_SENTENCE_LENGTH 100

constexpr unsigned int dataLength = 1500;
constexpr unsigned int cmdMaxCount = floor(dataLength/MINMEA_MAX_SENTENCE_LENGTH);
constexpr unsigned int  downloadTime = 1000;
constexpr unsigned char nmeaLength = 5;
constexpr unsigned char triesToUpdate = 5;

typedef float GpsFloat;
typedef BBTime GpsTime;

typedef enum {
	GpsOk = 0,
	GpsUnkonwnErr = 1,
	GpsNmeaNotFound = 2,
	GpsNoFix = 3
} GpsError;

/* Used to store raw data */
class GpsRawData {
public:
	GpsRawData();
	virtual ~GpsRawData() = 0;

	char data[dataLength];
};

/* Used to download data on user command */
class  GpsRawDataDownloader: public GpsRawData {//private GpsRawDataHolder {
public:
	GpsRawDataDownloader();
	~GpsRawDataDownloader();

	virtual GpsError refreshData() = 0;
};

/* Stores found commands */
class GpsCmds {
public:
	GpsCmds();
	virtual ~GpsCmds() = 0;
	char* nmeaMsg[cmdMaxCount];
	unsigned char nmeaMsgCount;
};

/* Finds commands */
class GpsCmdsFinder final: public GpsCmds {
public:
	GpsCmdsFinder(GpsRawData* rawData);
	~GpsCmdsFinder();

	unsigned char nmeaMsgSearch();

private:
	GpsRawData* rawData;
};

/* Used as a parent for final classes */
class GpsData {//private TinyEKF {
public:
	GpsData(GpsCmds* cmds);
	virtual ~GpsData();

	virtual GpsError updateParam() = 0;

	GpsError updateErr;

protected:
	GpsCmds* cmds;
};

/* Stores all NMEA data structs */
class GpsAllData final {
public:
	GpsAllData(GpsCmdsFinder* finder, GpsRawDataDownloader* downloader);
	~GpsAllData();

	GpsError update();
	GpsError add(GpsData* newData);

private:
	unsigned char dataCount;
	GpsCmdsFinder* finder;
	GpsData** data;
	GpsRawDataDownloader* downloader;
};

/* Stores GGA info */
class GpsGGA final: public GpsData {
public:
	GpsGGA(GpsCmds* cmds);
	~GpsGGA();

	GpsError updateParam() override;


	GpsFloat getLatitude();
	GpsFloat getLongitude();
	GpsTime getTime();

private:
	struct minmea_sentence_gga gga;

	GpsFloat latitude;
	GpsFloat longitude;

	GpsTime time;

	void setLL(GpsFloat * ll, const minmea_float latLong);
	void setTime();
	GpsError setLatLong();
};




