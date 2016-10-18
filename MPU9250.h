/*************************************************************************
 * 
 * CONICS LABS™ PROPRIETARY AND CONFIDENTIAL
 * __________________
 * 
 * Copyrights © [2016] - [2017]
 * Conics Labs Incorporated 
 * All Rights Reserved.
 * 
 * NOTICE:  All information contained herein is, and remains
 * the property of Conics Labs Incorporated and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to Conics Labs Incorporated
 * and its suppliers and may be covered by U.S. and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material via any medium
 * is strictly forbidden unless prior written permission is obtained
 * from Conics Labs Incorporated. If you have received this file in error, 
 * please notify copyright holder and destroy this and any other copies as instructed.
 *
 *************************************************************************/

#ifndef MPU9250_H
#define MPU9250_H

// Include the Register Map Definitions 
#include "MPU9250RegMap.h"

// To get exact-width (unit*_t) datatypes
#include <stdint.h>

// To output stuff on screen
#include <iostream>

// For I2C communication in RPi
#include "wiringPiI2C.h"

// To throw exceptions
#include <exception>
#include "MPU9250ExceptionsDefs.h"

// To generate delays
#include <sys/time.h>


#define DEBUG_MODE 0 //AKA Show me everything

// Class For MPU9250 related thingies.
class MPU9250
{
private:
	// Blah
	int fid_Magneto;
	int fid_AcceleroGyro;

public:
	
	MPU9250(); //Done
	
	int phyAdd2FID(uint16_t phyAdd); // Done
	void readByte(uint16_t devAddress, uint16_t regAddress, uint8_t* bucket2PutDataInto); // Done
	void readBytes(uint16_t devAddress, uint16_t regAddress, uint8_t noOfBytes2Read, uint8_t* bucket2PutDataInto); // Done
	void writeByte(uint16_t devAddress, uint16_t regAddress, uint8_t byte2Write); // Done
	
	void initAcceleroGyro(void);
	void initMagneto(void);

	void resetAcceleroGyro(void);
	void resetMagneto(void);

	uint16_t getMagnetoResoluiotn(void);
	uint16_t getAcceleroResolution(void);
	uint16_t getGyroResolution(void);
	
	void readMagnetoRawData(uint16_t* bucket2PutDataInto);
	void readAcceleroRawData(uint16_t* bucket2PutDataInto);
	void readGyroRawData(uint16_t* bucket2PutDataInto);
	void readTempData(uint16_t* bucket2PutDataInto);
	
	
	void calibrateAccelero(void);
	void calibrateGyro(void);
	void calibrateMagneto(void);

	bool selfTestMagneto(void);
	bool selfTestAccelero(void);
	bool selfTestGyro(void);

	unsigned long micros();
	unsigned long millis();
	void delay(unsigned long ms);

};



#endif 
//IFNDEF MPU9250_H