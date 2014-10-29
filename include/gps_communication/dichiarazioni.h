#ifndef DICHIARAZIONI_H
#define DICHIARAZIONI_H


#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <ros/types.h>
#include <std_msgs/Header.h>
#include <math.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/navdata_gps_channel.h>
#include "ardrone_autonomy/navdata_gps.h"
#include "ardrone_autonomy/navdata_altitude.h"
#include "ardrone_autonomy/Navdata.h"
#include <gps_communication/conversions.h>
#include "gps_communication/Coordinate_utm.h"
#include "ardrone_autonomy/navdata_demo.h"


#include <mission_planner_msgs/SensorPacket.h>
#include <mission_planner_msgs/Coordinate.h>
#include <mission_planner_msgs/CoordinateArray.h>



#define PI 3.14159265358979323846
#define YAW_ACCEPTED_ERROR 5 //massimo errore di orientamento accettato
#define ALTITUDE_ACCEPTED_ERROR 250 //massimo errore di altitudine accettato in mm
#define REFERENCE_ALTITUDE 2500 //misura in mm


typedef struct waypoints
	{
	double northing_struct;
	double easting_struct;
	} waypoints_UTM;

//prototipi funzioni
//-----------------------------------------------------------------------------------------------------------------------------------
void Land(void);
void Command_publisher(void);
void Reset(void);
void Distance_control(void);
void Control_forward(void);
void Interpolation(double Xs, double Ys, double Xf, double Yf);
void MessageReceived_LLgps(const ardrone_autonomy::navdata_gps& msg);
void Altitude_callback(const ardrone_autonomy::navdata_altitude& msg);
void Control_yaw(void);
void Control_altitude(void);
void MessageReceived_rotZ(const ardrone_autonomy::Navdata& msg);
void Positioning_in_hovering(void);
void Waypoints_topic(const mission_planner_msgs::CoordinateArrayConstPtr& msg);


#endif