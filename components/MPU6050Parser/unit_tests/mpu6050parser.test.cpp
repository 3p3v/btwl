/*
 * gps.test.cpp
 *
 *  Created on: Apr 7, 2023
 *      Author: esp32
 */
#include <catch2/catch_all.hpp>
#include <fff.h>
#include "../include/MPU6050Parser.hpp"

TEST_CASE("Parser", "[MPU6050Parser]")
{
	MPU6050 mpu = MPU6050();
	MPU6050Parser mpuParser(&mpu, 32768, 2);

	mpuParser.update();
	MPU6050Float z = mpuParser.getAccZ();
	REQUIRE(z.sign == 0);
	REQUIRE(z.unsign == 9763);
}

