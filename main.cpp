#include "sensor_service.h"

nsecs_t hzToNanosecs(int hz){
    return (nsecs_t)(1.0f / hz * 1000000000);
}

int main(int argc, char const *argv[])
{
    int channels[] ={0,1,2,3,4,5,6,7};
    int nChannel = 8;

    SensorSerivce service(channels, nChannel);
    service.open_device();
    service.start_service(hzToNanosecs(50));
    
    return 0;
}
