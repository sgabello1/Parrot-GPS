#include <gps_communication/dichiarazioni.h>
#include <ros/ros.h>

bool flag=true;

//-----------------------------------------------------------------------------------------------------------------------------------
void waypoints_topic(const mission_planner_msgs::CoordinateArrayConstPtr& msg)
	{
	std::ifstream myFile;		   
	std::string zoneWP;
	int index, numberWP;

	FILE* pFile;

	numberWP=msg->waypoint.size();
	ROS_INFO("Sono stati inseriti (%d) waypoints\n",numberWP);
	
	pFile = fopen ("/home/stefano/catkin_ws/src/gps_communication/src/points.txt","w");
   	for (index=0 ; index<numberWP ;index++)
   		{
    	fprintf (pFile, "latitude: %lf\n",msg->waypoint[index].latitude);
    	fprintf (pFile, "longitude: %lf\n",msg->waypoint[index].longitude);
    	fprintf (pFile, "altitude: %lf\n",msg->waypoint[index].altitude);
    	fprintf (pFile, "heading: %lf\n",msg->waypoint[index].heading);
    	fprintf (pFile, "-\n\n");
    	printf("waypoint number: %d\n", index);
    	printf ("latitude: %lf\n",msg->waypoint[index].latitude);
    	printf ("longitude: %lf\n",msg->waypoint[index].longitude);
    	printf ("altitude: %lf\n",msg->waypoint[index].altitude);
    	printf ("heading: %lf\n",msg->waypoint[index].heading);
    	printf ("-\n");
   		}
   	fclose (pFile);

   	ROS_INFO("Scrittura file completata\n");
   	flag=true;
   	return;
	}
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
	{
	ros::init(argc, argv, "read_waypoints");

	ros::NodeHandle n_subwp;
	ros::Subscriber sub1 = n_subwp.subscribe("gui_waypoints", 1000, waypoints_topic);

	ros::Rate loop_rate(100); //hz a cui gira il main
  	while(ros::ok())
     	{
     	if(flag==true)
     		{
     		printf("\nwaiting for a new route\n\n");
     		flag=false;
     		}
		ros::spinOnce();
		}
  	
    return 0;
	}
//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------