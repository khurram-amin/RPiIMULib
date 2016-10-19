#include "MPU9250.h"

int main()
{
	short *accel_data = new short[3];
	// accel_data[0] = 0;
	// accel_data[1] = 0;
	// accel_data[2] = 0;

	short *gyro_data = new short[3];
	// gyro_data[0] = 0;
	// gyro_data[1] = 0;
	// gyro_data[2] = 0;

	short *temp_data = new short[1];
	// temp_data[0] = 0;

	short *magneto_data = new short[3];
	// magneto_data[0] = 0;
	// magneto_data[1] = 0;
	// magneto_data[2] = 0;
	
	MPU9250 mpu9250;
	
	std::cout<<"config mpu2950 ";
	mpu9250.initAcceleroGyro();
	std::cout<<"done!"<<std::endl;
	std::cout<<std::endl;

	//mpu9250.delayMS(100);

	// std::cout<<"config AK8963 ";
	// mpu9250.initMagneto();
	// std::cout<<"done!"<<std::endl;


	
	while(1)
	{
		mpu9250.readAcceleroRawData(accel_data);

		// for(int i=0; i<3; i++)
		// {
		// 	std::cout<< i << ": " << 1.0f*accel_data[i] << std::endl;
		// }
		
		// mpu9250.readGyroRawData(gyro_data);

		// for(int i=0; i<3; i++)
		// {
		// 	std::cout<< i << ": " << 1.0f*gyro_data[i] << std::endl;
		// }

		// mpu9250.readTempRawData(temp_data);

		// std::cout<< "T" << ": " << *temp_data << std::endl;
		
		mpu9250.readMagnetoRawData(magneto_data);

		for(int i=0; i<3; i++)
		{
			std::cout<< i << ": " << 1.0f*magneto_data[i] << std::endl;
		}

		std::cout<<std::endl;
		std::cout<<std::endl;
		std::cout<<std::endl;
		std::cout<<std::endl;

		delay(1);
	}
}