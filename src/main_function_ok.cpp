#include <gps_communication/dichiarazioni.h>
#include <ros/ros.h>

bool flag_posizionamento_in_hovering=false;
bool Richiesta=false, flag_avanzamento1=false, flag_avanzamento2=true, flag_landing=false;
bool flag_waypoints=false;
int indexWP=0,spin=0,spin_altitude=0, numberWP=0;
double distance=10000000000,m_degree;
double northingGPS, eastingGPS, delta_yaw;
double RotZinX=2000, RotX, RotY, RotZ;
int altitude_reference;
double measured_altitude, instant_altitude; 


std::string zoneGPS, direction;
geometry_msgs::Twist comandi;
waypoints_UTM waypoints_UTM_vect[1000];

mission_planner_msgs::SensorPacket messaggio; 
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

	//ROS_INFO(" linear.x=(%lf)    linear.y=(%lf)    linear.z=(%lf)\n", comandi.linear.x, comandi.linear.y, comandi.linear.z);
	//ROS_INFO("angular.x=(%lf)   angular.y=(%lf)   angular.z=(%lf)\n", comandi.angular.x, comandi.angular.y, comandi.angular.z);
	pub.publish(comandi);
	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Reset(void)
	{
	ros::NodeHandle nh;
    geometry_msgs::Twist stop_wp;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	flag_avanzamento1=false;
	
	comandi.linear.x=0;
	comandi.linear.y=0;
	comandi.linear.z=0;
	comandi.angular.x=0;
	comandi.angular.y=0;
	comandi.angular.z=0;
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
			ROS_INFO("Way Point numero (%d) raggiunto, adesso vado al prossimo\n", indexWP);
		    ROS_INFO("Count Down:-------> 3\n");
		    sleep(1);
		    ROS_INFO("Count Down:-------> 2\n");
		    sleep(1);
		    ROS_INFO("Count Down:-------> 1\n");
		    sleep(1);
		    ROS_INFO("-------> GO to new way point!!!\n");
		    Richiesta=true;
			}
	    }
	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Control_forward(void)
	{
	if((flag_avanzamento1==true)&&(flag_avanzamento2==true))
		{
	    if(distance>20)
			comandi.linear.x=0.2;

		if((distance<=20)&&(distance>10))
			comandi.linear.x=0.1;

		if(distance<=10)
			comandi.linear.x=0.05;
		}
	else
		comandi.linear.x=0;

	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Interpolation(double Xs, double Ys, double Xf, double Yf)
	{
	double m,q;
	double Yapp,Xapp;

	m=(Yf-Ys)/(Xf-Xs);
	q=(Ys*Xf-Yf*Xs)/(Xf-Xs);
	//ROS_INFO("line equation: Y=(%lf)X+(%lf)\n",m,q);
	
	Yapp=pow((Yf-Ys),2);
	Xapp=pow((Xf-Xs),2);

	m_degree=(atan2((Yf-Ys),(Xf-Xs)))*(180/PI);
	distance=sqrt(Yapp+Xapp);
	ROS_INFO("distance:(%lf)meter     indexWP=(%d)\n",distance,indexWP);
	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void MessageReceived_LLgps(const ardrone_autonomy::navdata_gps& msg)
	{

	double lat = msg.lat_fused;
	double lng = msg.long_fused;
	messaggio.c_latit= lat;
	messaggio.c_longit=lng;
	
	//ROS_INFO("sto pubblicando il messaggio, lat=%lf e long=%lf", messaggio.c_latit, messaggio.c_longit);
	if(Richiesta)
		{
	    Richiesta=false;
	   	gps_common::LLtoUTM(msg.lat_fused, msg.long_fused, northingGPS, eastingGPS, zoneGPS);
	   	Interpolation(eastingGPS, northingGPS, waypoints_UTM_vect[indexWP].easting_struct, waypoints_UTM_vect[indexWP].northing_struct);
	   	}

	pub_feedback.publish(messaggio);  //per vederlo su mappa gui 	
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
	double yaw_mancante;
	
	if((RotZinX<(m_degree-YAW_ACCEPTED_ERROR))||(RotZinX>(m_degree+YAW_ACCEPTED_ERROR))) 
		{
		yaw_mancante=RotZinX-m_degree;

		if(yaw_mancante>180)				//calcolo per avere valori di yaw_mancante da -180 a +180 gradi
			yaw_mancante=-(360-yaw_mancante);
			
		if(yaw_mancante>0)
			spin=-1; //rotazione oraria
		else
			spin=1;	//rotazione oraria
			
		if((abs(yaw_mancante))>170)//sistemo i problemi del cambio di spin a +-180 facendo ruotare il drone per un po' di secondi
			{
			Command_publisher();
			sleep(0.5);	
			}

					

		if(abs(yaw_mancante)<40)	//scelta velocità rotazioni in base a distanza
			comandi.angular.z=0.1*spin;
		else
			comandi.angular.z=0.3*spin;
		}

    else
    	{
        comandi.angular.z=0;
        flag_avanzamento1=true;
		}
	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void Control_altitude(void)
	{
	double altitude_mancante;
	

	//controllo altitudine
    if((measured_altitude<(REFERENCE_ALTITUDE-ALTITUDE_ACCEPTED_ERROR))||(measured_altitude>(REFERENCE_ALTITUDE+ALTITUDE_ACCEPTED_ERROR))) 
			{

			altitude_mancante=measured_altitude-altitude_reference;
		
			if(altitude_mancante>0)
				spin_altitude=-1; 	//scende
			else
				spin_altitude=1;	//sale
			

			if(abs(altitude_mancante)<500)	//scelta velocità rotazioni in base a distanza
				comandi.linear.z=0.1*spin;
			else
				comandi.linear.z=0.1*spin;
			
			}
    else
    	{
        comandi.linear.z=0;
        flag_avanzamento2=true;
        //ROS_INFO("----> ACCEPTED ALTITUDE\n");
		}
	

	//se l'altezza del drone è minore di 1500mm, blocco l'avanzamento
	if(measured_altitude<1800)
		flag_avanzamento2=false;
		
		
	
	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
void MessageReceived_rotZ(const ardrone_autonomy::Navdata& msg)
	{
	RotZ=msg.rotZ;	
	if((msg.rotZ>=-180)&&(msg.rotZ<90))
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
	ROS_INFO("Count Down:-------> GO!!!\n");
	
	ROS_INFO("----> take off\n");
	pub_takeoff.publish(to);
    sleep(5);

    ROS_INFO("----> rise up\n");
	comandi.linear.z=0.3; //mi alzo per 3 secondi
	Command_publisher();
	sleep(3);

	ROS_INFO("----> hovering\n");
	Reset();
    sleep(1);

    flag_posizionamento_in_hovering=true;	

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
	ROS_INFO("Sono stati inseriti (%d) waypoints\n",numberWP);
	
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
   	Richiesta=true; //richiedo calcolo interpolazione
   	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------



//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
	{
	int main_counter=0, rate;
	double yaw_mancante;
	bool atterraggio=false;

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

	
	//posizionamento_in_hovering();	//take off e hovering
    //Richiesta=true; //richiedo calcolo interpolazione

	rate=20;
	ros::Rate loop_rate(rate); //hz a cui gira il main

  	while(ros::ok())
     	{
     	if(flag_waypoints==false)
			{
			printf("In attesa dei waypoints\n");
			sleep(1); //così ho tempo di leggere...
			}
	
		else
			{
			if(flag_posizionamento_in_hovering==false)
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
					Richiesta=true;	//richiedo ricalcolo interpolazione
					}

				if(flag_landing==true)
					{
					ROS_INFO("***************************\n");
		  			ROS_INFO("**  Programma Terminato  **\n");
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