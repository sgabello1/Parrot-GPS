#include <gps_communication/dichiarazioni.h>
#include <ros/ros.h>

bool flag_positioning_in_hovering=false;
bool request=false, flag_forward1=false, flag_forward2=true, flag_landing=false;
bool flag_waypoints=false;
int indexWP=0,spin=0,spin_altitude=0, numberWP=0;
double distance=10000000000,m_degree;
double northingGPS, eastingGPS;
double RotZinX=2000, RotX, RotY, RotZ;
int altitude_reference;
double measured_altitude, instant_altitude; 


std::string zoneGPS, direction;
geometry_msgs::Twist command;
waypoints_UTM waypoints_UTM_vect[1000];

mission_planner_msgs::SensorPacket message; 
ros::Publisher pub_feedback;

//-----------------------------------------------------------------------------------------------------------------------------------
void Land(void)
	{
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::Empty>("ardrone/land", 1000, true);

	std_msgs::Empty land_command;

	ROS_INFO("-------> landing\n");
	pub.publish(land_command);
	sleep(5);
	flag_landing=true;
	flag_waypoints=false;
	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Command_publisher(void)
	{
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000, true);
	//ROS_INFO(" linear.x=(%lf)    linear.y=(%lf)    linear.z=(%lf)\n", command.linear.x, command.linear.y, command.linear.z);
	//ROS_INFO("angular.x=(%lf)   angular.y=(%lf)   angular.z=(%lf)\n", command.angular.x, command.angular.y, command.angular.z);
	pub.publish(command);
	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Reset(void)
	{
	ros::NodeHandle nh;
    geometry_msgs::Twist stop_wp;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	flag_forward1=false;
	
	command.linear.x=0;
	command.linear.y=0;
	command.linear.z=0;
	command.angular.x=0;
	command.angular.y=0;
	command.angular.z=0;
	Command_publisher();
	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Distance_control(void)
	{
	if(distance<5)
		{
		Reset(); //hovering
		if(indexWP==(numberWP-1))
			{
			Land();
			return;
			}
		else
			{
			indexWP++;
			ROS_INFO("Reached way-point number (%d); I'm going to the next\n", indexWP);
		    ROS_INFO("Count Down:-------> 3\n");
		    sleep(1);
		    ROS_INFO("Count Down:-------> 2\n");
		    sleep(1);
		    ROS_INFO("Count Down:-------> 1\n");
		    sleep(1);
		    ROS_INFO("-------> GO to new way point!!!\n");
		    request=true;
			}
	    }
	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Control_forward(void)
	{
	if((flag_forward1==true)&&(flag_forward2==true))
		{
	    if(distance>20)
			command.linear.x=0.2;

		if((distance<=20)&&(distance>10))
			command.linear.x=0.1;

		if(distance<=10)
			command.linear.x=0.05;
		}
	else
		command.linear.x=0;

	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Interpolation(double Xs, double Ys, double Xf, double Yf)
	{
	double m,q;
	double Ytmp,Xtmp;

	m=(Yf-Ys)/(Xf-Xs);
	q=(Ys*Xf-Yf*Xs)/(Xf-Xs);
	//ROS_INFO("line equation: Y=(%lf)X+(%lf)\n",m,q);
	
	Ytmp=pow((Yf-Ys),2);
	Xtmp=pow((Xf-Xs),2);

	m_degree=(atan2((Yf-Ys),(Xf-Xs)))*(180/PI);
	distance=sqrt(Ytmp+Xtmp);
	ROS_INFO("distance from waypoint:(%lf)meter\n",distance);
	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void MessageReceived_LLgps(const ardrone_autonomy::navdata_gps& msg)
	{

	double lat = msg.lat_fused;
	double lng = msg.long_fused;
	message.c_latit= lat;
	message.c_longit=lng;
	
	//ROS_INFO("sto pubblicando il message, lat=%lf e long=%lf", message.c_latit, message.c_longit);
	if(request)
		{
	    request=false;
	   	gps_common::LLtoUTM(msg.lat_fused, msg.long_fused, northingGPS, eastingGPS, zoneGPS);
	   	Interpolation(eastingGPS, northingGPS, waypoints_UTM_vect[indexWP].easting_struct, waypoints_UTM_vect[indexWP].northing_struct);
	   	}

	pub_feedback.publish(message);  //per vederlo su mappa gui 	
	return;
 	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Altitude_callback(const ardrone_autonomy::navdata_altitude& msg)
	{
	measured_altitude=msg.altitude_raw;
	return;
 	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Control_yaw(void)
	{
	double yaw_missing;
	
	if((RotZinX<(m_degree-YAW_ACCEPTED_ERROR))||(RotZinX>(m_degree+YAW_ACCEPTED_ERROR))) 
		{
		yaw_missing=RotZinX-m_degree;

		if(yaw_missing>180)				//calcolo per avere valori di yaw_mancante da -180 a +180 gradi
			yaw_missing=-(360-yaw_missing);
			
		if(yaw_missing>0)
			spin=-1; //rotazione oraria
		else
			spin=1;	//rotazione oraria
			
		if((abs(yaw_missing))>170)//sistemo i problemi del cambio di spin a +-180 facendo ruotare il drone per un po' di secondi
			{
			Command_publisher();
			sleep(0.5);	
			}

					

		if(abs(yaw_missing)<40)	//scelta velocità rotazioni in base a distanza
			command.angular.z=0.1*spin;
		else
			command.angular.z=0.3*spin;
		}

    else
    	{
        command.angular.z=0;
        flag_forward1=true;
		}
	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Control_altitude(void)
	{
	double altitude_missing;
	

	//controllo altitudine
    if((measured_altitude<(REFERENCE_ALTITUDE-ALTITUDE_ACCEPTED_ERROR))||(measured_altitude>(REFERENCE_ALTITUDE+ALTITUDE_ACCEPTED_ERROR))) 
			{

			altitude_missing=measured_altitude-altitude_reference;
		
			if(altitude_missing>0)
				spin_altitude=-1; 	//scende
			else
				spin_altitude=1;	//sale
			

			if(abs(altitude_missing)<500)	//scelta velocità rotazioni in base a distanza
				command.linear.z=0.1*spin;
			else
				command.linear.z=0.1*spin;
			
			}
    else
    	{
        command.linear.z=0;
        flag_forward2=true;
        //ROS_INFO("----> ACCEPTED ALTITUDE\n");
		}
	

	//se l'altezza del drone è minore di 1500mm, blocco l'avanzamento
	if(measured_altitude<1800)
		flag_forward2=false;
		
		
	
	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void MessageReceived_rotZ(const ardrone_autonomy::Navdata& msg)
	{
	RotZ=msg.rotZ;	
	if((msg.rotZ>=-180)&&(msg.rotZ<90)) //remap di rotz (che è fatto rispetto al nord) in rotzinx (che è fatto rispetto ad est)
 		RotZinX=msg.rotZ+90; 
 	else
 		RotZinX=msg.rotZ-270;

	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Positioning_in_hovering(void)
	{
	ros::NodeHandle nhtakeoff, nh;
	std_msgs::Empty to;	
    
    ros::Publisher pub_takeoff = nhtakeoff.advertise<std_msgs::Empty> ("ardrone/takeoff", 1000,true);

	Reset();
	
	ROS_INFO("Count Down:-------> 3\n");
	sleep(1);
	ROS_INFO("Count Down:-------> 2\n");
	sleep(1);
	ROS_INFO("Count Down:-------> 1\n");
	sleep(1);
	ROS_INFO("----> GO!!!\n");
	
	ROS_INFO("----> take off\n");
	pub_takeoff.publish(to);
    sleep(5);

    ROS_INFO("----> rise up\n");
	command.linear.z=0.3; //mi alzo per 3 secondi
	Command_publisher();
	sleep(3);

	ROS_INFO("----> hovering\n");
	Reset();
    sleep(1);

    flag_positioning_in_hovering=true;	

	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Waypoints_topic(const mission_planner_msgs::CoordinateArrayConstPtr& msg)
	{
	std::string zoneWP;
	int index;
	double northingWP, eastingWP;

	numberWP=msg->waypoint.size();
	ROS_INFO("Inserted (%d) waypoints\n",numberWP);
	
	for (index=0 ; index<numberWP ;index++)
   		{
    	gps_common::LLtoUTM(msg->waypoint[index].latitude, msg->waypoint[index].longitude, northingWP, eastingWP, zoneWP);
		waypoints_UTM_vect[index].northing_struct= northingWP;
		waypoints_UTM_vect[index].easting_struct= eastingWP;
		}
   	for (index=0 ; index<numberWP ;index++)
   		{
   		printf ("waypoint number: %d\n", index);
    	printf ("latitude: %lf\n",msg->waypoint[index].latitude);
    	printf ("longitude: %lf\n",msg->waypoint[index].longitude);
    	printf ("altitude: %lf\n",msg->waypoint[index].altitude);
    	printf ("heading: %lf\n",msg->waypoint[index].heading);
    	printf ("-\n");
   		}

   	flag_waypoints=true;
   	request=true; //richiedo calcolo interpolazione
   	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------



//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
	{
	int main_counter=0, rate;
	

  	ros::init(argc, argv, "main_function");

	ros::NodeHandle n_sub_LLgps;
	ros::NodeHandle n_sub_rotZ;
	ros::NodeHandle n_sub_altitude;
	ros::NodeHandle nh;

    	
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1000);
	ros::Subscriber sub1 = n_sub_LLgps.subscribe("ardrone/navdata_gps", 1, MessageReceived_LLgps);	
    ros::Subscriber sub2 = n_sub_rotZ.subscribe("ardrone/navdata", 1, MessageReceived_rotZ);	
    ros::Subscriber sub3 = n_sub_altitude.subscribe("ardrone/navdata_altitude", 1, Altitude_callback);

    ros::NodeHandle n_subwp;
	ros::Subscriber sub4 = n_subwp.subscribe("gui_waypoints", 1000, Waypoints_topic);

	ros::NodeHandle nh1;
	pub_feedback = nh1.advertise<mission_planner_msgs::SensorPacket> ("feedback", 1,true);	

	
	rate=20;
	ros::Rate loop_rate(rate); //hz a cui gira il main

  	while(ros::ok())
     	{
     	if(flag_waypoints==false)
			{
			printf("waiting for waypoints from GUI\n");
			sleep(1); //così ho tempo di leggere...
			}
	
		else
			{
			if(flag_positioning_in_hovering==false)
				{
				Positioning_in_hovering();
				}
			else
				{
			
				Control_yaw();
				Control_altitude();
				Control_forward();
				Distance_control();
				Command_publisher();

				main_counter++;

				if(main_counter==(rate*4)) //sono passati 4secondi
					{
					main_counter=0;
					request=true;	//richiedo ricalcolo interpolazione
					}

				if(flag_landing==true)
					{
					ROS_INFO("***************************\n");
		  			ROS_INFO("**   Program completed   **\n");
		  			ROS_INFO("***************************\n");
					return 0;
					}
				}
			}
			
		ros::spinOnce();
		}

  	return 0;
	}
//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------