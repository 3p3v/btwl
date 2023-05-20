/*
 * bbformats.test.cpp
 *
 *  Created on: Apr 29, 2023
 *      Author: esp32
 */
#include <catch2/catch_all.hpp>
#include <fff.h>
#include "BBFormats.hpp"
#include <cstring>

TEST_CASE("Positive BBFloat to string", "[BBFormats]")
{
	BBFloat bbFloat(1, 29958);

	char arr[8] {};
	arr[7] = '\n';

	int length = bbFloat.bbFloatToString(arr);

	REQUIRE(length == 7);
	REQUIRE(strcmp(arr, "1.29958") == 0);
}

TEST_CASE("Negative BBFloat to string", "[BBFormats]")
{
	BBFloat bbFloat(false, 133, 79458);

	char arr[11] {};
	arr[10] = '\n';

	int length = bbFloat.bbFloatToString(arr);

	REQUIRE(length == 10);
	REQUIRE(strcmp(arr, "-133.79458") == 0);
}

