#include "MPU9250.h"

int main()
{
	short *data = new short[3];
	
	MPU9250 mpu9250;
	
	cout<<"config mpu2950 ";
	mpu9250.initMPU9250();
	cout<<"done!"<<endl;
	
	while(1)
	{
		mpu9250.readAccelData(data);

		for(int i=0; i<3; i++)
		{
			std::cout<< i << ": " << 1.0f*data[i] << std::endl;
		}
		std::cout<<std::endlstd::<<endlstd::<<endl;
		delay(10);
	}
}