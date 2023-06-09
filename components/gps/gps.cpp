/*
 * gps.cpp
 *
 *  Created on: Apr 5, 2023
 *      Author: esp32
 */

#include <cstdlib>
#include <cstring>
#include "include/gps.hpp"
#include <cstdio>
#include "gps.hpp"

GpsRawData::GpsRawData() {
	//ifChanged = true;
	// data = (char*)pvPortMalloc(sizeof(char*) * dataLength);
}

GpsRawData::~GpsRawData() {
	// vPortFree(data);
}

GpsRawDataDownloader::GpsRawDataDownloader() {}

GpsRawDataDownloader::~GpsRawDataDownloader() {}

GpsCmds::GpsCmds()
	: nmeaMsgCount(0) {}

GpsCmds::~GpsCmds() {}

GpsCmdsFinder::GpsCmdsFinder(GpsRawData* rawData)
	: rawData(rawData) {}

GpsCmdsFinder::~GpsCmdsFinder() {}

/* find all commands in input buffer */
unsigned char GpsCmdsFinder::nmeaMsgSearch() {
	nmeaMsgCount = 0;

	for(int i = 0; i < dataLength - MINMEA_MAX_SENTENCE_LENGTH; i++) {
		if(nmeaMsgCount == cmdMaxCount)
			return nmeaMsgCount;
		if(*(rawData->data + i) == '$') {
			*(nmeaMsg + nmeaMsgCount) = (rawData->data + i);
			nmeaMsgCount++;
		}
	}

	return nmeaMsgCount;
}

GpsData::GpsData(GpsCmds* cmds)
	: updateErr(GpsOk) {
	this->cmds = cmds;
};

GpsData::~GpsData() {};

GpsAllData::GpsAllData(GpsCmdsFinder* finder, GpsRawDataDownloader* downloader)
	: dataCount(0), data(nullptr){
	this->finder = finder;
	this->downloader = downloader;
}

GpsAllData::~GpsAllData() {
	if(dataCount != 0)
		vPortFree(data);
}

GpsError GpsAllData::update() {
	// Get fresh data
	GpsError err = GpsOk;

	downloader->refreshData();
	finder->nmeaMsgSearch();

	for(unsigned char i = 0; i < dataCount; i++) {
		for(unsigned char j = 0; j < triesToUpdate; j++) {
			if(data[i]->updateParam() == GpsOk) {
				break;
			} else if(j == triesToUpdate - 1) {
				err = GpsNmeaNotFound;
			}else {
				downloader->refreshData();
				finder->nmeaMsgSearch();
			}
		}
	}

	return err;
}

GpsError GpsAllData::add(GpsData* newData) {
	GpsData* temp[dataCount];

	for(unsigned char i = 0; i < dataCount; i++)
		temp[i] = data[i];

	if(dataCount != 0)
		vPortFree(data);

	data = (GpsData**)pvPortMalloc(sizeof(GpsData**) * (dataCount + 1));

	for(unsigned char i = 0; i < dataCount; i++) {
		data[i] = temp[i];
	}

	data[dataCount] = newData;

	dataCount++;

	return GpsOk;
}

GpsGGA::GpsGGA(GpsCmds* cmds)
	: GpsData(cmds) {}

GpsGGA::~GpsGGA() {}

GpsError GpsGGA::updateParam() {
	for(unsigned char i = 0; i < cmds->nmeaMsgCount; i++) {
		if(minmea_sentence_id((*(cmds->nmeaMsg + i)), false) == MINMEA_SENTENCE_GGA) {
			if(minmea_parse_gga(&gga, *(cmds->nmeaMsg + i))) {
				if(gga.latitude.value == 0 || gga.longitude.value == 0) {
					updateErr = GpsNoFix;
				} else {
					updateErr = GpsOk;

					setLatLong();
					setTime();
				}

				return updateErr;
			}
		}
	}

	updateErr = GpsNmeaNotFound;
	return updateErr;
}

void GpsGGA::setTime() {
	time.hh = (unsigned char)gga.time.hours;
	time.mm = (unsigned char)gga.time.minutes;
	time.ss = (unsigned char)gga.time.seconds;
}

GpsError GpsGGA::setLatLong() {
	if(gga.latitude.value == 0 || gga.longitude.value == 0)
		return GpsNmeaNotFound;

	latitude = minmea_tocoord(&gga.latitude);
	longitude = minmea_tocoord(&gga.longitude);

	return GpsOk;
}

GpsFloat GpsGGA::getLatitude() {
	return latitude;
}

GpsFloat GpsGGA::getLongitude() {
	return longitude;
}

GpsTime GpsGGA::getTime()
{
    return time;
}
