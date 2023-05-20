/*
 * accel.cpp
 *
 *  Created on: Apr 13, 2023
 *      Author: esp32
 */

#include <cmath>
#include "include/MPU6050Parser.hpp"

MPU6050Parser::MPU6050Parser(MPU6050 * mpu, const int scale, const int max)
	: scale(scale), max(max) {
	this->mpu = mpu;
}

MPU6050Parser::~MPU6050Parser() {}

MPU6050Float MPU6050Parser::getAccX() {
	return accX;
}

MPU6050Float MPU6050Parser::getAccY() {
	return accY;
}

MPU6050Float MPU6050Parser::getAccZ() {
	return accZ;
}

void MPU6050Parser::setScale(const int scale, const int max) {
	this->scale = scale;
	this->max = max;
}

// void MPU6050Parser::setAcc(MPU6050Float * xyz, const int reading) {
// 	if(((float)reading * max / scale) < 0)
// 		xyz->positive = false;
// 	else
// 		xyz->positive = true;

// 	xyz->sign = abs((int)((float)reading * max / scale));
// 	xyz->unsign = abs((int)(((float)((float)reading * max / scale) - (float)xyz->sign) * MPU6050_ACC_MULTIPLIER));
// }

void MPU6050Parser::update() {
	accX = (float)mpu->getAccelerationX() * max / scale;
	accY = (float)mpu->getAccelerationY() * max / scale;
	accZ = (float)mpu->getAccelerationZ() * max / scale;
}


// MPU6050Int MPU6050Parser::getInterruptCause() {
// 	if(mpu->getIntFreefallStatus())
// 		return MPU6050FreeFall;
// 	else
// 		return MPU6050Clk;
// }
