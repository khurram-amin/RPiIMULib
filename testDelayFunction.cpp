// To generate delays
#include <sys/time.h>

// To output stuff on screen
#include <iostream>

// Unix Sleep function
#include <unistd.h>

//#define DEBUG_MODE

// This function will get current time in micro seconds.
unsigned long micros()
{
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return 1000000 * tv.tv_sec + tv.tv_usec;
  
}


// This function will get current time in milli senconds.
unsigned long millis()
{
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return 1000 * tv.tv_sec + tv.tv_usec/1000;
}


// This function will generate a delay in milli seconds.
void delayMS(unsigned long ms)
{
  unsigned long start = micros();

  usleep(ms*1000);

  unsigned long end = micros();

  float dleayTime = (end - start)/1000;

  #ifdef DEBUG_MODE
    std::cout << "Generated a delay of " << (float) dleayTime << " milli seconds." << std::endl;
  #endif
}


int main ()
{
  while(1)
  {
    // mpu9250.readAcceleroRawData(data);

    // for(int i=0; i<3; i++)
    // {
    //  std::cout<< i << ": " << 1.0f*data[i] << std::endl;
    // }
    // std::cout<<std::endl;
    // std::cout << millis() << std::endl;
    delay(10);
    // usleep(10000);
    // std::cout << millis() << std::endl;
  }
}