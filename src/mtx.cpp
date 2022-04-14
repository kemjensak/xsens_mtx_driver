#include <ros/ros.h>
#include <stdio.h>		
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <curses.h>
#include <string>
#include <iostream>

#include "cmtdef.h"
#include "xsens_time.h"
#include "xsens_list.h"
#include "cmtscan.h"
#include "cmt3.h"
#include "mtx.h"


// this macro tests for an error and exits the program with a message if there was one
#define EXIT_ON_ERROR(res,comment) if (res != XRV_OK) { printw("Error %d occurred in " comment ": %s\n",res,xsensResultText(res)); exit(1); }

using namespace xsens;

inline int isnum(int c)
{
        return (c >= '0' && c <= '9');
}                                     

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mtx_driver");
	ros::NodeHandle nh;
	(void) argc; (void) argv;	// Make the compiler stop complaining about unused parameters
	xsens::Cmt3 cmt3;
	int userQuit = 0;	
	unsigned long mtCount = 0;
	int temperatureOffset = 0;
	int screenSensorOffset = 0;
	CmtDeviceId deviceIds[256];
	
	CmtOutputMode mode;
	CmtOutputSettings settings;

	XsensResultValue res = XRV_OK;
	short screenSkipFactor = 10;
	short screenSkipFactorCnt = screenSkipFactor;

	initscr();
	noecho();
	nodelay(stdscr, 1);
	keypad(stdscr, 1);
	raw();

	// Perform hardware scan
	mtCount = 0;
	
	std::cout<<"\nwaiting for mtx device\n";
	mtCount = doHardwareScan(cmt3, deviceIds);

	// int chr = getch();
	// if (chr == 3 || chr == 'q') {
	// 	userQuit = 1;
	// 	// break;
	// }
	
	// ros::Duration(1).sleep();
	
	// if (mtCount == 0){
	// 	ros::shutdown();
	// }
	// predefined mode and settings
	mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
	// settings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER;
	settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
	// Set device to user input settings
	doMtSettings(cmt3, mode, settings, deviceIds);
	refresh();
	screenSensorOffset = calcScreenOffset(mode, settings, screenSensorOffset);
	writeHeaders(mtCount, mode, settings, temperatureOffset, screenSensorOffset);
	// vars for sample counter & temp.
	unsigned short sdata;
	double tdata;
	
	//structs to hold data.
	CmtCalData caldata;
	CmtQuat qat_data;
	CmtEuler euler_data;
	CmtMatrix matrix_data;

	// Initialize packet for data
	Packet* packet = new Packet((unsigned short) mtCount, cmt3.isXm());
	int y, x;
	getyx(stdscr, y, x);
	x = 0;
	move(y, x);

	double eulerData[5][3];
	double AccelerationData[5][3];
	double GyroscopeData[5][3];

	double** Acc_data = new double* [5];
	double** Ori_data = new double* [5];
	double** Gyr_data = new double* [5];

	for (int i = 0; i < 5; i++)
	{
		Acc_data[i] = new double[3];
		Ori_data[i] = new double[3];
		Gyr_data[i] = new double[3];
	}

	// publisher
	ros::Publisher imu_pub_[3] = {nh.advertise<sensor_msgs::Imu>("/imu_1", 100),
									nh.advertise<sensor_msgs::Imu>("/imu_2", 100),
									nh.advertise<sensor_msgs::Imu>("/imu_3", 100)};
	
	// ros::Publisher cal_imu_pub_[3] = {nh.advertise<sensor_msgs::Imu>("/cal_imu_1", 100),
	// 								nh.advertise<sensor_msgs::Imu>("/cal_imu_2", 100),
	// 								nh.advertise<sensor_msgs::Imu>("/cal_imu_3", 100)};
	
	
	while (!userQuit && res == XRV_OK && ros::ok()) 
	{
		XsensResultValue result = cmt3.waitForDataMessage(packet);
		if(result != XRV_OK)
		{
			if( (result == XRV_TIMEOUTNODATA) || (result == XRV_TIMEOUT) )
				continue;  //Ignore the error and restart the while loop			

			echo();
			endwin();
			delete packet;
			cmt3.closePort();
			printf("\nError %d occured in waitForDataMessage, can not recover.\n", result);
			exit(1);
		}

		//get sample count, goto position & display.
		sdata = packet->getSampleCounter();
		mvprintw(y, x, "Sample Counter %05hu\n", sdata);

		if (screenSkipFactorCnt++ != screenSkipFactor) {
			//continue;
		}
		screenSkipFactorCnt = 0;
		
		for (unsigned int i = 0; i < mtCount; i++) {	
			// Output Temperature
			if ((mode & CMT_OUTPUTMODE_TEMP) != 0) {					
				tdata = packet->getTemp(i);
				mvprintw(y + 4 + i * screenSensorOffset, x, "%6.2f", tdata);
			}
			move(y + 5 + temperatureOffset + i * screenSensorOffset, x);
			if ((mode & CMT_OUTPUTMODE_CALIB) != 0) {					
				caldata = packet->getCalData(i);
				mvprintw(y + 5 + temperatureOffset + i * screenSensorOffset, x, 
						"%6.2f\t%6.2f\t%6.2f", 	caldata.m_acc.m_data[0], 
						caldata.m_acc.m_data[1], caldata.m_acc.m_data[2]);
				mvprintw(y + 7 + temperatureOffset + i * screenSensorOffset, x, 
						"%6.2f\t%6.2f\t%6.2f", caldata.m_gyr.m_data[0], 
						caldata.m_gyr.m_data[1], caldata.m_gyr.m_data[2]);
				mvprintw(y + 9 + temperatureOffset + i * screenSensorOffset, x,
						"%6.2f\t%6.2f\t%6.2f",caldata.m_mag.m_data[0], 
						caldata.m_mag.m_data[1], caldata.m_mag.m_data[2]);
				move(y + 13 + temperatureOffset + i * screenSensorOffset, x);

				imu_data.angular_velocity.x = caldata.m_gyr.m_data[0];
				imu_data.angular_velocity.y = caldata.m_gyr.m_data[1];
				imu_data.angular_velocity.z = caldata.m_gyr.m_data[2];

				imu_data.linear_acceleration.x = caldata.m_acc.m_data[0];
				imu_data.linear_acceleration.y = caldata.m_acc.m_data[1];
				imu_data.linear_acceleration.z = caldata.m_acc.m_data[2];

				Acc_data[i][0] = caldata.m_acc.m_data[0];
				Acc_data[i][1] = caldata.m_acc.m_data[1];
				Acc_data[i][2] = caldata.m_acc.m_data[2];
				Gyr_data[i][0] = caldata.m_gyr.m_data[0];
				Gyr_data[i][1] = caldata.m_gyr.m_data[1];
				Gyr_data[i][2] = caldata.m_gyr.m_data[2];

				
				
			}

			if ((mode & CMT_OUTPUTMODE_ORIENT) == 0) {
				continue;
			}
			int yt, xt;
			getyx(stdscr, yt, xt);
			switch (settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {
			case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
				// Output: quaternion
				qat_data = packet->getOriQuat(i);
				mvprintw(yt++, xt, "%6.3f\t%6.3f\t%6.3f\t%6.3f", qat_data.m_data[0], 
						qat_data.m_data[1], qat_data.m_data[2], qat_data.m_data[3]);

				imu_data.orientation.x = qat_data.m_data[1];
				imu_data.orientation.y = qat_data.m_data[2];
				imu_data.orientation.z = qat_data.m_data[3];
				imu_data.orientation.w = qat_data.m_data[0];
				

				break;

			case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
				// Output: Euler
				euler_data = packet->getOriEuler(i);
				mvprintw(yt++, xt, "%6.1f\t%6.1f\t%6.1f", euler_data.m_roll, 
						euler_data.m_pitch, euler_data.m_yaw);

				imu_data.orientation.x = euler_data.m_roll;
				imu_data.orientation.y = euler_data.m_pitch;
				imu_data.orientation.z = euler_data.m_yaw;

				Ori_data[i][0] = euler_data.m_roll;
				Ori_data[i][1] = euler_data.m_pitch;
				Ori_data[i][2] = euler_data.m_yaw;

				break;

			
			default:
				break;
			}

			imu_data.header.stamp = ros::Time::now();
			imu_data.header.frame_id = "world";
			imu_data.orientation_covariance[0] = 0.01745;
			imu_data.orientation_covariance[4] = 0.01745;
			imu_data.orientation_covariance[8] = 0.15708;
			imu_data.angular_velocity_covariance[0] = 0.0004;
			imu_data.angular_velocity_covariance[4] = 0.0004;
			imu_data.angular_velocity_covariance[8] = 0.0004;
			imu_data.linear_acceleration_covariance[0] = 0.0004;
			imu_data.linear_acceleration_covariance[4] = 0.0004;
			imu_data.linear_acceleration_covariance[8] = 0.0004;

			

			imu_pub_[i].publish(imu_data);

			move(yt, xt);
			refresh();
		}
		

		
	
	}

	



	echo();
	endwin();
	delete packet;
	ros::shutdown();
	cmt3.closePort();
	return 0;
}


//////////////////////////////////////////////////////////////////////////
// doHardwareScan
//
// Checks available COM ports and scans for MotionTrackers
int doHardwareScan(xsens::Cmt3 &cmt3, CmtDeviceId deviceIds[])
{
	XsensResultValue res;
	List<CmtPortInfo> portInfo;
	unsigned long portCount = 0;
	int mtCount;
	
	// printw("Scanning for connected Xsens devices...");
	xsens::cmtScanPorts(portInfo);
	portCount = portInfo.length();
	// printw("done\n");

	if (portCount == 0) {
		std::cout << "No MotionTrackers found\n\n";
		return 0;
	}

	for(int i = 0; i < (int)portCount; i++) {	
		printw("Using COM port %s at ", portInfo[i].m_portName);
		
		switch (portInfo[i].m_baudrate) {
		case B9600  : printw("9k6");   break;
		case B19200 : printw("19k2");  break;
		case B38400 : printw("38k4");  break;
		case B57600 : printw("57k6");  break;
		case B115200: printw("115k2"); break;
		case B230400: printw("230k4"); break;
		case B460800: printw("460k8"); break;
		case B921600: printw("921k6"); break;
		default: printw("0x%lx", portInfo[i].m_baudrate);
		}
		printw(" baud\n\n");
	}

	printw("Opening ports...");
	//open the port which the device is connected to and connect at the device's baudrate.
	for(int p = 0; p < (int)portCount; p++){
		res = cmt3.openPort(portInfo[p].m_portName, portInfo[p].m_baudrate);
		EXIT_ON_ERROR(res,"cmtOpenPort");  
	}
	printw("done\n\n");

	 //set the measurement timeout to 100ms (default is 16ms)
	int timeOut = 100;
	res = cmt3.setTimeoutMeasurement(timeOut);
	EXIT_ON_ERROR(res, "set measurment timeout");
	printf("Measurement timeout set to %d ms\n", timeOut);

	//get the Mt sensor count.
	printw("Retrieving MotionTracker count (excluding attached Xbus Master(s))\n");
	mtCount = cmt3.getMtCount();
	printw("MotionTracker count: %d\n\n", mtCount);

	// retrieve the device IDs 
	printw("Retrieving MotionTrackers device ID(s)\n");
	for(int j = 0; j < mtCount; j++){
		res = cmt3.getDeviceId((unsigned char)(j+1), deviceIds[j]);
		EXIT_ON_ERROR(res,"getDeviceId");
		printw("Device ID at busId %i: %08lx\n\n",j+1,(long) deviceIds[j]);
	}
	
	return mtCount;
}


//////////////////////////////////////////////////////////////////////////
// doMTSettings
//
// Set user settings in MTi/MTx
// Assumes initialized global MTComm class
void doMtSettings(xsens::Cmt3 &cmt3, CmtOutputMode &mode, 
		CmtOutputSettings &settings, CmtDeviceId deviceIds[]) 
{
	XsensResultValue res;
	unsigned long mtCount = cmt3.getMtCount();

	// set sensor to config sate
	res = cmt3.gotoConfig();
	EXIT_ON_ERROR(res,"gotoConfig");

	unsigned short sampleFreq;
	sampleFreq = cmt3.getSampleFrequency();

	// set the device output mode for the device(s)
	printw("Configuring your mode selection\n");

	for (unsigned int i = 0; i < mtCount; i++) {
		CmtDeviceMode deviceMode(mode, settings, sampleFreq);
		if ((deviceIds[i] & 0xFFF00000) != 0x00500000) {
			// not an MTi-G, remove all GPS related stuff
			deviceMode.m_outputMode &= 0xFF0F;
		}
		res = cmt3.setDeviceMode(deviceMode, true, deviceIds[i]);
		EXIT_ON_ERROR(res,"setDeviceMode");
	}

	// start receiving data
	res = cmt3.gotoMeasurement();
	EXIT_ON_ERROR(res,"gotoMeasurement");
}

//////////////////////////////////////////////////////////////////////////
// writeHeaders
//
// Write appropriate headers to screen
void writeHeaders(unsigned long mtCount, CmtOutputMode &mode, 
		CmtOutputSettings &settings, int &temperatureOffset, 
		int &screenSensorOffset)
{
	int y, x;
	getyx(stdscr, y, x);
	for (unsigned int i = 0; i < mtCount; i++) {
		mvprintw(y + 2 + i * screenSensorOffset, x, "MotionTracker %d\n", i + 1);
		if ((mode & CMT_OUTPUTMODE_TEMP) != 0) {
			temperatureOffset = 3;
			mvprintw(y + 3 + i * screenSensorOffset, x, "Temperature");
			mvprintw(y + 4 + i * screenSensorOffset, 7, "degrees celcius\n");
			move(y + 6 + i * screenSensorOffset, x);
		}

		if ((mode & CMT_OUTPUTMODE_CALIB) != 0) {
			mvprintw(y + 3 + temperatureOffset + i * screenSensorOffset, x, 
					"Calibrated sensor data");
			mvprintw(y + 4 + temperatureOffset + i * screenSensorOffset, x, 
					" Acc X\t Acc Y\t Acc Z");
			mvprintw(y + 5 + temperatureOffset + i * screenSensorOffset, x + 23, 
					"(m/s^2)");
			mvprintw(y + 6 + temperatureOffset + i * screenSensorOffset, x, 
					" Gyr X\t Gyr Y\t Gyr Z");
			mvprintw(y + 7 + temperatureOffset + i * screenSensorOffset, x + 23, 
					"(rad/s)");
			mvprintw(y + 8 + temperatureOffset + i * screenSensorOffset, x, 
					" Mag X\t Mag Y\t Mag Z");
			mvprintw(y + 9 + temperatureOffset + i * screenSensorOffset, x + 23, 
					"(a.u.)");
			move(y + 11 + temperatureOffset + i * screenSensorOffset, x);
		}

		if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {
			printw("Orientation data\n");
			switch(settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {
			case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
				printw("    q0\t    q1\t    q2\t    q3\n");
				break;
			case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
				printw("  Roll\t Pitch\t   Yaw\n");
				printw("                       degrees\n");
				break;
			case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
				printw(" Matrix\n");
				break;
			default:
				;
			}			
		}

		if ((mode & CMT_OUTPUTMODE_POSITION) != 0) {
			printw("\nLongitude\tLatitude\t Altitude\n");
		}
	}
	move(y, x);
	refresh();
}

//////////////////////////////////////////////////////////////////////////
// calcScreenOffset
//
// Calculates offset for screen data with multiple MTx on Xbus Master
int calcScreenOffset(CmtOutputMode &mode, CmtOutputSettings &settings, 
		int screenSensorOffset)
{
    // 1 line for "Sensor ..."
    screenSensorOffset += 1;
    if ((mode & CMT_OUTPUTMODE_TEMP) != 0)
        screenSensorOffset += 3;
    if ((mode & CMT_OUTPUTMODE_CALIB) != 0)
        screenSensorOffset += 8;
    if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {
        switch(settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {
        case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
            screenSensorOffset += 4;
            break;
        case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
            screenSensorOffset += 4;
            break;
        case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
            screenSensorOffset += 6;
            break;
        default:
            ;
        }
    }

	if ((mode & CMT_OUTPUTMODE_POSITION) != 0) {
		screenSensorOffset += 4;
	}
	return screenSensorOffset;
}

