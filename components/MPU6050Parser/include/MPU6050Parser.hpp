/*
 * accel.hpp
 *
 *  Created on: Apr 13, 2023
 *      Author: esp32
 */

#pragma once

#include "MPU6050.h"
#include "BBFormats.hpp"

#define MPU6050_DIGITS 			5
#define MPU6050_ACC_MULTIPLIER	10000

typedef enum {
	MPU6050Clk = 0,
	MPU6050Impact = 1,
	MPU6050FreeFall = 2
} MPU6050Int;

typedef float MPU6050Float;

class MPU6050Parser {
public:
	/*	max = maximum acceleration in [g] */
	MPU6050Parser(MPU6050 * mpu, const int scale, const int max);
	~MPU6050Parser();

	void update();

	// MPU6050Int getInterruptCause();

	void setScale(const int scale, const int max);

	MPU6050Float getAccX();
	MPU6050Float getAccY();
	MPU6050Float getAccZ();

private:
	MPU6050 * mpu;
	/* max value on scale */
	int scale;
	int max;

	MPU6050Float accX;
	MPU6050Float accY;
	MPU6050Float accZ;

	void setAcc(MPU6050Float * xyz, const int reading);
};
