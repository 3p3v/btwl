/*
 * gps.test.cpp
 *
 *  Created on: Apr 7, 2023
 *      Author: esp32
 */
#include <catch2/catch_all.hpp>
#include <fff.h>
#include "../include/gps.hpp"
//#include "../include/gps_esp8266.hpp"
#include <cstdlib>

//FAKE_VALUE_FUNCTION(unsigned int, test);

class  GpsFakeDownloader final: public GpsRawDataDownloader {
public:
	GpsFakeDownloader() {};
	~GpsFakeDownloader() {};

	GpsError refreshData() {
		char cmd[] = "14:10:25  $GNGGA,141025.000,5059.14238,N,01714.06348,E,1,06,3.00,127.5,M,42.0,M,,*7C\n14:10:25  $GPGSA,A,3,18,16,27,02,,,,,,,,,3.14,3.00,0.95,1*1F\n14:10:25  $BDGSA,A,3,13,11,,,,,,,,,,,3.14,3.00,0.95,4*00\n14:10:25  $GPGSV,2,1,08,29,82,38,22,18,68,65,22,16,61,277,33,27,39,290,23*7B\n14:10:25  $GPGSV,2,2,08,2,28,61,26,5,20,79,23,15,14,66,,23,1,354,19*79\n14:10:25  $BDGSV,1,1,04,13,38,47,15,11,27,78,22,5,24,136,,2,8,109,*5D";
			for(int i = 0; i < dataLength; i++)
				data[i] = cmd[i];

		return GpsOk;
	}
};

TEST_CASE("Basic Parser Use", "[GpsGGA]")
{
	GpsFakeDownloader downloader;
	GpsCmdsFinder finder(&downloader);
	GpsGGA gga(&finder);

	downloader.refreshData();
	finder.nmeaMsgSearch();
	REQUIRE(gga.updateParam() == GpsOk);
	GpsFloat lalitude = gga.getLatitude();
	GpsFloat longitude = gga.getLongitude();

	REQUIRE(lalitude.sign == 50);
	REQUIRE(lalitude.unsign == 9857063);
	REQUIRE(longitude.sign == 17);
	REQUIRE(longitude.unsign == 2343913);
}

TEST_CASE("Update All", "[GpsGGA]")
{
	GpsFakeDownloader downloader;
	GpsCmdsFinder finder(&downloader);
	GpsGGA gga(&finder);
	GpsGGA gga2(&finder);

	GpsAllData all(&finder, &downloader);
	all.add(&gga);
	all.add(&gga2);

	REQUIRE(all.updateAllParams() == GpsOk);

	GpsFloat lalitude = gga.getLatitude();
	GpsFloat longitude = gga.getLongitude();

	REQUIRE(lalitude.sign == 50);
	REQUIRE(lalitude.unsign == 9857063);
	REQUIRE(longitude.sign == 17);
	REQUIRE(longitude.unsign == 2343913);

	lalitude = gga2.getLatitude();
	longitude = gga2.getLongitude();

	REQUIRE(lalitude.sign == 50);
	REQUIRE(lalitude.unsign == 9857063);
	REQUIRE(longitude.sign == 17);
	REQUIRE(longitude.unsign == 2343913);
}
