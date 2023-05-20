/*
 * BBFormats.h
 *
 *  Created on: Apr 27, 2023
 *      Author: esp32
 */

#pragma once

#define	BBFLOAT_DEFAULT_LEN		10
#define	BBFLOAT_DEFAULT_PREC	3

class BBFloat {
public:
	BBFloat();
	BBFloat(const unsigned int sign, const unsigned int unsign);
	BBFloat(const bool positive, const unsigned int sign, const unsigned int unsign);
	~BBFloat();

	bool positive;
	unsigned int sign;
	unsigned int unsign;

	/* Length of a given array must be at least = signLen + unsignLen + 3 */
	int bbFloatToString(char * arr, const unsigned int signLen, const unsigned int unsignLen);
	int bbFloatToString(char * arr);
	void floatToBBFloat(float reading, const int precision);
	void floatToBBFloat(float reading);
};

class BBTime final {
public:
	BBTime();
	~BBTime();
	unsigned char hh;
	unsigned char mm;
	unsigned char ss;
};
