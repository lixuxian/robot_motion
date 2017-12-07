#include "SensorListener.h"
#include "data_sender.h"
#include "data_dumper.h"  // add by lixuxian 2017/06/27
#include "utils/timers.h" // add by lixuxian 2017/06/29
#include "bt_send.h"
#include <stdio.h>
#include <sstream>
#include <string>
#include <string.h>
using namespace std;

void SensorListener::onSensorChanged(sensors_event_t& event){
	char sensor_data[128];
	sprintf(sensor_data,"%d#%.2f,%.2f,%.2f",event.channel, event.data[0], event.data[1], event.data[2]);
    //printf("sensor listener:%.2f, %.2f, %.2f\n", event.data[0], event.data[1], event.data[2]);
    send(sensor_data);
    //printf("%s\n", sensor_data);
}
char sensor_data_string[1024];

// add accData and gyroData by lixuxian 2017.12.07
char imu_data_string[1024]; 
void SensorListener::onMultiSensorChanged(sensors_event_t *events, sensors_event_t *accData, sensors_event_t *gyroData, int nEvents){
    stringstream ss;
    stringstream acc_gyro;
    string sensor_data;
    string imu_data;
    sensors_event_t *event;
    sensors_event_t *acc; // add ccc
    sensors_event_t *gyro; // add gyro
    int i=0;
    for(; i<nEvents; i++){
        event = events+i;
        acc = accData + i;
        gyro = gyroData + i;
        ss<< event->channel << "#" 
            << event->data[0] << "," << event->data[1] << "," << event->data[2] << ";";
        acc_gyro << event->channel << "#"
            << acc->data[0] << "," << acc->data[1] << "," << acc->data[2] << ","
            << gyro->data[0] << "," << gyro->data[1] << "," << gyro->data[2] << ";";            
    }

    ss >> sensor_data;
    const int len = sensor_data.length();
    strncpy(sensor_data_string, sensor_data.c_str(), len);
    sensor_data_string[len] = '\0';
    send(sensor_data_string);
    // send_bt(sensor_data_string);
    dump("data/data_dumper_test.txt", sensor_data_string); //add by lixuxian 2017/06/27

    // dump acc, gyro to imu_data.txt
    acc_gyro >> imu_data;
    const int imu_data_len = imu_data.length();
    strncpy(imu_data_string, imu_data.c_str(), imu_data_len);
    imu_data_string[imu_data_len] = '\0';
    dump("data/imu_data.txt", imu_data_string);
} 
