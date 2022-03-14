#include "sensor_msgs/Imu.h"


#ifndef __EXAMPLE_LINUX_H
#define __EXAMPLE_LINUX_H
void mySigintHandler(int sig);
int doHardwareScan(xsens::Cmt3 &, CmtDeviceId []);
void doMtSettings(xsens::Cmt3 &, CmtOutputMode &, CmtOutputSettings &, CmtDeviceId []);
void getUserInputs(CmtOutputMode &, CmtOutputSettings &);
void writeHeaders(unsigned long, CmtOutputMode &, CmtOutputSettings &, int&, int&);
int calcScreenOffset(CmtOutputMode &, CmtOutputSettings &, int);

sensor_msgs::Imu imu_data;
sensor_msgs::Imu cal_imu_data;
	
clock_t t_now,t_start;

    

#endif
