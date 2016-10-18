#include "MPU9250.h"


// Class constructor. It will
//					1. Open MPU9250 as Linux File for communication
//					2. Open AK8963 as Linux File for communication
MPU9250::MPU9250()
{

	Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
	Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
	Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
	Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR 

	fid_Magneto = wiringPiI2CSetup(AK8963_ADDRESS);
	fid_AcceleroGyro = wiringPiI2CSetup(MPU9250_ADDRESS);

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
int MPU9250::phyAdd2FID(uint8_t phyAdd)
{
	int phy2FID;
	if (phyAdd == (int)MPU9250_ADDRESS)
	{
		phy2FID = fid_AcceleroGyro;

		#if DEBUG_MODE
		std::cout << "Physical Address to File ID translation for MPU9250. " << std::hex << MPU9250_ADDRESS  << " converted into " << std::hex << fid_AcceleroGyro << "."<< std::endl;
		#endif

		return phy2FID;
	}	
	else if (phyAdd == (int)AK8963_ADDRESS)
	{
		phy2FID = fid_Magneto;

		#if DEBUG_MODE
		std::cout << "Physical Address to File ID translation for AK8963. " << std::hex << AK8963_ADDRESS  << " converted into " << std::hex << fid_Magneto << "." << std::endl;
		#endif

		return phy2FID;
	}
	else
		throw UNKNOWN_DEVICE_PHYSICAL_ADDRESS;
}


// This function will return 1-byte read from regAddress of device devAddress
char MPU9250::readByte(uint8_t devAddress, uint8_t regAddress)
{
	return wiringPiI2CReadReg8( phyAdd2FID(devAddress), regAddress);
}

void MPU9250::readBytes(uint8_t devAddress, uint8_t regAddress, uint8_t noOfBytes2Read, uint8_t* bucket2PutDataInto)
{
	char data;
	for (char i = 0; i < noOfBytes2Read; i++)
	{
		data = readByte(devAddress, regAddress+i);
		bucket2PutDataInto[i] = data;
		//std::cout << (int) data << std::endl;
	}
}

void MPU9250::writeByte(uint8_t devAddress, uint8_t regAddress, uint8_t bucket2PutDataInto)
{
	int phy2FID = phyAdd2FID(devAddress);
	wiringPiI2CWriteReg8(phy2FID, regAddress, bucket2PutDataInto);
}

// Write One-byte of data at regAddress of device devAddress.
// void MPU9250::writeByte(uint16_t devAddress, uint16_t regAddress, uint8_t byte2Write)
// {
// 	/// Convert Physical Address of Device into FileID
// 	int phy2FID = phyAdd2FID(devAddress);

// 	int write_done = wiringPiI2CWriteReg8(phy2FID, (int)regAddress, (int)byte2Write);

// 	#if DEBUG_MODE
// 	std::cout << "I just wrote " << std::hex << byte2Write  << " on register " << regAddress << " of device " << phy2FID << " with result: " << write_done << "."<< std::endl;
// 	#endif
// }

// Initilize MPU9250 device
void MPU9250::initAcceleroGyro()
{
	// uint8_t Gscale = GFS_250DPS;
	// uint8_t Ascale = AFS_2G;
	// Wake the device up.
	// (D7=0=No-Reset-Internel-registers + D6=0=No-Sleep + D5=0=No-Cycling + D4=0=Gyro-Sense-Path-Connected + D3=0=Power-Up-PTAT-voltage-generator-&-PTAT-ADC + D2,D1,D0=000=INTERNAL-OSCILATOR-20-MHz )
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
	delayMS(100); //Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	// get stable time source
	// Set Clock source to be PLL
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);

	// Configure Gyro and Accelerometer
	//(D7=X + D6=0=Overwrite-FIFO-when-it-gets-full + D5:D3=000=Disable-FSYNCH + D2:D0=011=GyroBW-41Hz-GyroDelay-5.9ms-Fs-1KHz-TempBW-42Hz-TempDelay-4.8ms)
	writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

	// Set sample rate
	//Use a 200 Hz rate; the same rate set in CONFIG above
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	//uint8_t gyro_old_config;
	//readByte (MPU9250_ADDRESS, GYRO_CONFIG, &gyro_old_config);
	uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
	//Clear self-test bits [7:5] 
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0xE0 );
	// Clear AFS bits [4:3]
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18);
	// Set full scale range for the gyro
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3);


	// Set accelerometer configuration
	//uint8_t accel_old_config;
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
	// Clear self-test bits [7:5] 
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0xE0);
	// Clear AFS bits [4:3]
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG,  c & ~0x18);
	// Set full scale range for the accelerometer 
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3);

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
	// Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F );
	// Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03);
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting


	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	// Enable data ready (bit 0) interrupt
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);

}

// This function will read three 16-bit registers corresponding to RAW accelerometer readings
void MPU9250::readAcceleroRawData(uint16_t* bucket2PutDataInto)
{
	uint8_t noOfBytes2Read = 6;
	uint8_t* rawData = new uint8_t[noOfBytes2Read];
	for (uint8_t i = 0; i < noOfBytes2Read; i++)
	{
		rawData[i] = 0;
	}
	
	readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, noOfBytes2Read, &rawData[0]);
	
	bucket2PutDataInto[0] = (uint16_t)(( ((uint16_t)rawData[0]) << 8) | rawData[1]);
	bucket2PutDataInto[1] = (uint16_t)(( ((uint16_t)rawData[2]) << 8) | rawData[3]);
	bucket2PutDataInto[2] = (uint16_t)(( ((uint16_t)rawData[4]) << 8) | rawData[5]);
}


// This function will read three 16-Bit registers correspoding to the RAW gyroscope readings
void MPU9250::readGyroRawData(uint16_t* bucket2PutDataInto)
{
	// Define and initilize an array to store register values.
	uint8_t noOfBytes2Read = 6;
	uint8_t* rawData = new uint8_t[noOfBytes2Read];
	for (uint8_t i = 0; i < noOfBytes2Read; i++)
	{
		rawData[i] = 0;
	}

	
	readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, noOfBytes2Read, &rawData[0]);
	
	bucket2PutDataInto[0] = (uint16_t)(( ((uint16_t)rawData[0]) << 8) | rawData[1]);
	bucket2PutDataInto[1] = (uint16_t)(( ((uint16_t)rawData[2]) << 8) | rawData[3]);
	bucket2PutDataInto[2] = (uint16_t)(( ((uint16_t)rawData[4]) << 8) | rawData[5]);

}


// This function will read 16-bit register corresponding to the RAW temperature sensor.
void MPU9250::readTempRawData(uint16_t* bucket2PutDataInto)
{
	uint8_t noOfBytes2Read = 2;
	uint8_t* rawData = new uint8_t[noOfBytes2Read];  // x/y/z gyro register data stored here
	for (uint8_t i = 0; i < noOfBytes2Read; i++)
	{
		rawData[i] = 0;
	}

	readBytes(MPU9250_ADDRESS, TEMP_OUT_H, noOfBytes2Read, &rawData[0]);

	bucket2PutDataInto[0] = (uint16_t)( ((uint16_t)rawData[0]) << 8 | rawData[1] );
}


// This function will read three 16-bit registers correspoding to the RAW magnetometer sensor.
void MPU9250::readMagnetoRawData(uint16_t* bucket2PutDataInto)
{
	// Define and initilize an array to store register values.
	uint8_t noOfBytes2Read = 7;
	uint8_t rawData[7];// = new uint8_t[noOfBytes2Read];
	for (uint8_t i = 0; i < noOfBytes2Read; i++)
	{
		rawData[i] = 0;
	}

	// uint8_t ST2_reg = 0;
	// readByte(AK8963_ADDRESS, AK8963_ST1, &ST2_reg); 

	// if ( ST2_reg & 0x01 )
	// {

	// 	readBytes(AK8963_ADDRESS, AK8963_XOUT_L, noOfBytes2Read, &rawData[0]);

	// 	std::cout << "here " << (int)rawData[6] << std::endl;

	// 	if (!((int)rawData[6] & 0x08))
	// 	{

	// 		bucket2PutDataInto[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
	// 		bucket2PutDataInto[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]);  // Data stored as little Endian
	// 		bucket2PutDataInto[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]);
	// 	}
	// }
	std::cout << "HERE" << std::endl;
	if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)
	{ // wait for magnetometer data ready bit to be set
		std::cout << "HERE2" << std::endl;
			readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
			uint8_t c = rawData[6]; // End data read by reading ST2 register

			if (!(c & 0x08)) 
			{ // Check if magnetic sensor overflow set, if not then report data
				std::cout << "HERE3" << std::endl;
				bucket2PutDataInto[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
				bucket2PutDataInto[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]);  // Data stored as little Endian
				bucket2PutDataInto[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]);
			}
	}
}

// This function will reset Accelerometer + Gyroscope to their default state
void MPU9250::resetAcceleroGyro(void)
{
	uint8_t setState = (uint8_t)0x80;
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, setState);
	delayMS(100);
}


// This function will initiate the Magnetometer Sensor
void MPU9250::initMagneto(void)
{
	// First extract the factory calibration for each magnetometer axis

	uint8_t noOfBytes2Read = 3;
	uint8_t rawData[3];
	for (uint8_t i = 0; i < noOfBytes2Read; i++)
	{
		rawData[i] = 0;
	}

	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
	delayMS(10);
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); 
	delayMS(10);

	readBytes(AK8963_ADDRESS, AK8963_ASAX, noOfBytes2Read, &rawData[0]);  // Read the x-, y-, and z-axis calibration values

	//destination[0] = (float)(rawData[0] - 128) / 256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	//destination[1] = (float)(rawData[1] - 128) / 256.0f + 1.0f;
	//destination[2] = (float)(rawData[2] - 128) / 256.0f + 1.0f;
	
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
	delayMS(10);

	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
	delayMS(50);
}

// This function will get current time in micro seconds.
unsigned long MPU9250::micros()
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	unsigned long time_in_micros = 1000000 * tv.tv_sec + tv.tv_usec;
	return time_in_micros;
}


// This function will get current time in milli senconds.
unsigned long MPU9250::millis()
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	unsigned long time_in_millis = 1000 * tv.tv_sec + tv.tv_usec/1000;
	return time_in_millis;
}


// This function will generate a delay in milli seconds.
void MPU9250::delayMS(unsigned long ms)
{
	unsigned long start = micros();
  usleep(ms*1000); // Sleep
  unsigned long end = micros();


  #if DEBUG_MODE
  float dleayTime = (end - start)/1000;
  std::cout << "Generated a delay of " << (float) dleayTime << " milli seconds." << std::endl;
  #endif
}











