#include <gps_communication/dichiarazioni.h>
#include <ros/ros.h>

float magx, magy;


//-----------------------------------------------------------------------------------------------------------------------------------
void MessageReceived(const ardrone_autonomy::Navdata& msg)
	{
	magx=msg.magX;
	magy=msg.magY;
	
	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void MessageReceived_degree(const ardrone_autonomy::navdata_gps& msg)
	{
	//ROS_INFO("degree= %f 	degree_magnetic= %f\n", msg.degree, msg.degree_magnetic);
	return;
 	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void MessageReceived_north(const ardrone_autonomy::navdata_demo& msg)
	{
	//ROS_INFO("attitude= %f\n", msg.psi);
	return;
 	}
//-----------------------------------------------------------------------------------------------------------------------------------



//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
	{
	
	ros::init(argc, argv, "main_function_2");

	ros::NodeHandle  nh2, nh3;
	ros::NodeHandle n_sub_rotZ;
		
	ros::Subscriber sub3 = nh3.subscribe("ardrone/navdata_demo", 1, MessageReceived_north);	
    ros::Subscriber sub1 = nh2.subscribe("ardrone/navdata_gps", 1, MessageReceived_degree);	
    ros::Subscriber sub2 = n_sub_rotZ.subscribe("ardrone/navdata", 1, MessageReceived);
	
	

	//ros::Rate loop_rate(1); //hz a cui gira il main
  	while(ros::ok())
     	{  
     	ROS_INFO("maggx= %f		maggy= %f\n", magx, magy);
     	
		ros::spinOnce();
		}
  	
    return 0;
	}
//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------