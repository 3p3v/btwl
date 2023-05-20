/*
 * BBFormats.cpp
 *
 *  Created on: Apr 27, 2023
 *      Author: esp32
 */

#include "include/BBFormats.hpp"
#include <stdio.h>
#include <math.h>

BBFloat::BBFloat()
	: positive(true), sign(0), unsign(0) {}

BBFloat::BBFloat(const unsigned int sign, const unsigned int unsign)
	: positive(true), sign(sign), unsign(unsign) {}

BBFloat::BBFloat(const bool positive, const unsigned int sign, const unsigned int unsign)
	: positive(positive), sign(sign), unsign(unsign) {}

BBFloat::~BBFloat() {}

int BBFloat::bbFloatToString(char * arr, const unsigned int signLen, const unsigned int unsignLen) {
	int i = 0;

	if(positive == false) {
		*arr = '-';
		i++;
	}

	i += snprintf(arr + i, signLen + 1, "%i", sign); //signLen + 1

	arr[i] = '.';
	i++;

	i += snprintf(arr + i, unsignLen + 1, "%i", unsign); // unsignLen + 1

	return i;
}

int BBFloat::bbFloatToString(char * arr) {
	return bbFloatToString(arr, BBFLOAT_DEFAULT_LEN, BBFLOAT_DEFAULT_LEN);
}

BBTime::BBTime()
	: hh(0), mm(0), ss(0) {}

BBTime::~BBTime() {}

void BBFloat::floatToBBFloat(float reading, const int precision) {
	if(reading < 0) {
        positive = false;
        reading *= -1;
    }

    sign = (int)reading;
    unsign = (int)((float)(reading - (float)sign) * (float)pow(10, precision));
}

void BBFloat::floatToBBFloat(float reading) {
	floatToBBFloat(reading, BBFLOAT_DEFAULT_PREC);
}