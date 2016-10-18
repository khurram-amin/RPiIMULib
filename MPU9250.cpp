#include "MPU9250.h"


// Class constructor. It will
//					1. Open MPU9250 as Linux File for communication
//					2. Open AK8963 as Linux File for communication
MPU9250::MPU9250()
{
	fid_AcceleroGyro = wiringPiI2CSetup(MPU9250_ADDRESS);
	fid_Magneto = wiringPiI2CSetup(AK8963_ADDRESS);

	#if DEBUG_MODE
	// Show what are you going to do, as you do it
		if (fid_AcceleroGyro != -1)
			std::cout << "Opened /dev/I2C file for communication with AcceleroGyro. File ID is: " << fid_AcceleroGyro << std::endl;
		else
			std::cout << "Couldn't open /dev/I2C file for communication with AcceleroGyro." << std::endl;

		if (fid_Magneto != -1)
			std::cout << "Opened /dev/I2C file for communication with Magneto. File ID is: " << fid_Magneto << std::endl;
		else
			std::cout << "Couldn't open /dev/I2C file for communication with Magneto." << std::endl;
	#endif

	if (fid_AcceleroGyro==-1) 
		throw MPU9250_FILE_OPENING_ERROR;
	if (fid_Magneto == -1) 
		throw AK8963_FILE_OPENING_ERROR;
}


// Converts Physical Address of I2C device into FileID for wiringPI
int MPU9250::phyAdd2FID(uint16_t phyAdd)
{
	int phy2FID;
	if (phyAdd == (int)MPU9250_ADDRESS)
	{
		phy2FID = fid_AcceleroGyro;

		#ifdef DEBUG_MODE
			std::cout << "Physical Address to File ID translation for MPU9250. " << MPU9250_ADDRESS  << " converted into " << fid_AcceleroGyro "."<< std::endl;
		#endif

		return phy2FID;
	}	
	else if (phyAdd == (int)AK8963_ADDRESS)
	{
		phy2FID = fid_Magneto;

		#ifdef DEBUG_MODE
			std::cout << "Physical Address to File ID translation for AK8963. " << AK8963_ADDRESS  << " converted into " << fid_Magneto << "." << std::endl;
		#endif

		return phy2FID;
	}
	else
		throw UNKNOWN_DEVICE_PHYSICAL_ADDRESS;
}


// This function will read one byte from the the addressed device's (in devAddress) register (mentioned in regAddress) over I2C and put it into bucket2PutDataInto
void MPU9250::readByte(uint16_t devAddress, uint16_t regAddress, uint8_t* bucket2PutDataInto)
{
	// Convert Physical Address of Device into FileID
	int phy2FID = phyAdd2FID(devAddress);
	// Read Byte from register
	bucket2PutDataInto = (uint8_t)wiringPiI2CReadReg8(phy2FID, (int)regAddress);
	#ifdef DEBUG_MODE
		std::cout << "I just read " << bucket2PutDataInto  << " from register " << regAddress << " of device " << phy2FID << "."<< std::endl;
	#endif
}


// This function will read multiple bytes of data starting from regAddress + [0 to noOfBytes2Read] and put it into bucket2PutDataInto.
void MPU9250::readBytes(uint16_t devAddress, uint16_t regAddress, uint8_t noOfBytes2Read, uint8_t* bucket2PutDataInto)
{
	// Iterate noOfBytes2Read of bytes times, calling readByte each time to read one single byte and incrementing the address.
	for (uint8_t i = 0; i < noOfBytes2Read; i++)
	{
		readByte(devAddress, regAddress+i, bucket2PutDataInto[i]);
	}
	#ifdef DEBUG_MODE
		std::cout << "I just read " << noOfBytes2Read  << " bytes from " << regAddress << " of device " << phy2FID << "."<< std::endl;
		std::cout << "WAllah, those were alot of reads. Why would you do that to me, Priya! ? " << std::endl;
		std::cout << "You better be doing something good with all those reads otherwise I am going to call Gull Khan! " << std::endl;
	#endif
}


// Write One-byte of data at regAddress of device devAddress.
void MPU9250::writeByte(uint16_t devAddress, uint16_t regAddress, uint8_t byte2Write)
{
	/// Convert Physical Address of Device into FileID
	int phy2FID = phyAdd2FID(devAddress);

	int write_done = wiringPiI2CWriteReg8(phy2FID, (int)regAddress, (int)byte2Write);

	#ifdef DEBUG_MODE
		std::cout << "I just wrote " << byte2Write  << " on register " << regAddress << " of device " << phy2FID << " with result: " << write_done << "."<< std::endl;
	#endif
}

// Initilize MPU9250 device
void MPU9250::initAcceleroGyro()
{
	uint8_t MScale = 1;
	uint8_t GScale = 0;
	uint8_t AScale = 0;
	// Wake the device up.
	uint8_t setState = (uint8_t)0x00;// (D7=0=No-Reset-Internel-registers + D6=0=No-Sleep + D5=0=No-Cycling + D4=0=Gyro-Sense-Path-Connected + D3=0=Power-Up-PTAT-voltage-generator-&-PTAT-ADC + D2,D1,D0=000=INTERNAL-OSCILATOR-20-MHz )
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, setState);
	delay(100); //Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	// get stable time source
	setState = (uint8_t)0x01; // Set Clock source to be PLL
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, setState);

	// Configure Gyro and Accelerometer
	setState = (uint8_t)0x03; //(D7=X + D6=0=Overwrite-FIFO-when-it-gets-full + D5:D3=000=Disable-FSYNCH + D2:D0=011=GyroBW-41Hz-GyroDelay-5.9ms-Fs-1KHz-TempBW-42Hz-TempDelay-4.8ms)
	writeByte(MPU9250_ADDRESS, CONFIG, setState);

	// Set sample rate
	setState = (uint8_t)0x04; //Use a 200 Hz rate; the same rate set in CONFIG above
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, setState);

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t gyro_old_config;
	readByte (MPU9250_ADDRESS, GYRO_CONFIG, &gyro_old_config);
	
	setState = (uint8_t)(gyro_old_config & ~0xE0); // Clear self-test bits [7:5] 
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, setState);

	setState = (uint8_t)(gyro_old_config & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, setState);

	setState = (uint8_t)(gyro_old_config | Gscale << 3); // Set full scale range for the gyro
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, setState);


	// Set accelerometer configuration
	uint8_t accel_old_config;
	readByte(MPU9250_ADDRESS, ACCEL_CONFIG, &accel_old_config);

	setState = (uint8_t)(accel_old_config & ~0xE0); // Clear self-test bits [7:5] 
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, setState);

	setState = (uint8_t)(accel_old_config & ~0x18);  // Clear AFS bits [4:3]
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, setState);

	setState = (uint8_t)(accel_old_config | Ascale << 3); // Set full scale range for the accelerometer 
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, setState);

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	readByte(MPU9250_ADDRESS, ACCEL_CONFIG2, &accel_old_config);

	setState = (uint8_t)(accel_old_config & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, setState );

	setState = (uint8_t)(accel_old_config | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, setState);
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting


	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
	// can join the I2C bus and all can be controlled by the Arduino as master
	setState = (uint8_t)0x22;
	writeByte(MPU9250_ADDRESS, INT_PIN_CFG, setState);

	setState = (uint8_t)0x01;  // Enable data ready (bit 0) interrupt
	writeByte(MPU9250_ADDRESS, INT_ENABLE, setState);

}

// This function will read three 16-bit registers corresponding to RAW accelerometer readings
void MPU9250::readAcceleroRawData(uint16_t* bucket2PutDataInto)
{
	uint8_t rawData[6];
	uint8_t noOfBytes2Read = 6;
	readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, noOfBytes2Read, &rawData);
	bucket2PutDataInto[0] = (uint16_t)(((uint16_t)rawData[0] << 8) | rawData[1]);
	bucket2PutDataInto[1] = (uint16_t)(((uint16_t)rawData[2] << 8) | rawData[3]);
	bucket2PutDataInto[2] = (uint16_t)(((uint16_t)rawData[4] << 8) | rawData[5]);
}














