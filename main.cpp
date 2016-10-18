#include "MPU9250.h"

int main()
{
	uint16_t *data = new uint16_t[3];
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	
	MPU9250 mpu9250;
	
	std::cout<<"config mpu2950 ";
	
	mpu9250.initAcceleroGyro();
	
	std::cout<<"done!"<<std::endl;
	
	while(1)
	{
		mpu9250.readAcceleroRawData(data);

		for(int i=0; i<3; i++)
		{
			std::cout<< i << ": " << 1.0f*data[i] << std::endl;
		}
		std::cout<<std::endl;

		mpu9250.readGyroRawData(data);

		for(int i=0; i<3; i++)
		{
			std::cout<< i << ": " << 1.0f*data[i] << std::endl;
		}
		std::cout<<std::endl;
		std::cout<<std::endl;
		std::cout<<std::endl;
		std::cout<<std::endl;

		mpu9250.delayMS(10);
	}
}