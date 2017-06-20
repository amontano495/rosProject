#include <string>
#include <math.h>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <csignal>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

//Global gps coords of the robot
double botLat;
double botLon;

//Sets the current mode of the robot (MANUAL, AUTO, etc);
void setBotMode( std::string mode, ros::NodeHandle &node );

//Sets the global coords to what the GPS device responds
void getBotCoords( const sensor_msgs::NavSatFix& msg );

//Determines if the bot is within the given two vertices
//The two vertices represent opposite corners of a given square
bool boundaryCheck( double V1lat, double V1lon, double V2lat, double V2lon );

int main(int argc, char** argv)
{
	//Initial ROS settings
	ros::init(argc, argv, "erle_rand_walker");
	ros::NodeHandle n;
	int rate = 10;
	ros::Rate r(rate);

	//times to move and reverse
	time_t moveTime;
	time_t revTime;

	//randomized variables for nondeterministic movement
	int randomDirection;
	int randomTime;


	//Dictate the throttle and steering of the robot
	ros::Publisher rcOverridePub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1 );
	mavros_msgs::OverrideRCIn msg_override;

	//Used to gain information from GPS device
	ros::Subscriber gpsSub = n.subscribe("/mavros/global_position/global", 1000, &getBotCoords );

	//fixed time from user input
	int inputMoveTime = atoi(argv[1]);

	//Represents two vertices of opposite corners of a square
	double boundaryNElat = 39.539152;
	double boundaryNElon = -119.814124;
	double boundarySWlat = 39.537778;
	double boundarySWlon = -119.814219;

	//Sets robot to "MANUAL" mode
	setBotMode( "MANUAL" , n );

	while(ros::ok())
	{
		randomDirection = rand() % 2;
		randomTime = rand() % 3;

		if( randomDirection == 1 ) 
		{
			ROS_INFO("TURNING RIGHT...");
			//Force wheels right
			msg_override.channels[0] = 1100;
			rcOverridePub.publish(msg_override);
			ros::spinOnce();

			revTime = time(NULL) + randomTime;
			while( time(NULL) < revTime )
			{
				//reverse
				msg_override.channels[2] = 1552;
				rcOverridePub.publish(msg_override);
				ros::spinOnce();
				r.sleep();
			}
			ROS_INFO("TURNING COMPLETED");
		}

		if( randomDirection == 0 )
		{
			ROS_INFO("TURNING LEFT...");
			//Force wheels left
			msg_override.channels[0] = 1900;
			rcOverridePub.publish(msg_override);
			ros::spinOnce();

			revTime = time(NULL) + randomTime;
			while( time(NULL) < revTime )
			{
				//reverse
				msg_override.channels[2] = 1552;
				rcOverridePub.publish(msg_override);
				ros::spinOnce();
				r.sleep();
			}
			ROS_INFO("TURNING COMPLETED");
		}

		
		//return wheels to forward position
		msg_override.channels[0] = 1500;
		rcOverridePub.publish(msg_override);
		ros::spinOnce();

		//move forward for fixed time or until out of bounds
		ROS_INFO("MOVING...");
		moveTime = time(NULL) + inputMoveTime;
		while( time(NULL) < moveTime || boundaryCheck(boundaryNElat, boundaryNElon, boundarySWlat, boundarySWlon) )
		{
			//forward
			msg_override.channels[2] = 1425;
			rcOverridePub.publish(msg_override);
			ros::spinOnce();
			r.sleep();
		}

		ros::spinOnce();
	}

	return 0;
}

bool boundaryCheck( double V1lat, double V1lon, double V2lat, double V2lon )
{
	bool botWithinBoundary = true;
	if(V2lat < botLat && botLat < V1lat )
	{
		if( V2lon < botLon && botLon < V1lon )
			botWithinBoundary = true;
		else
			botWithinBoundary = false;
	}
	else
	{
		botWithinBoundary = false;
	}

	return botWithinBoundary;
}

void getBotCoords( const sensor_msgs::NavSatFix &msg )
{
	botLat = msg.latitude;
	botLon = msg.longitude;
}

void setBotMode( std::string mode, ros::NodeHandle &node)
{
	mavros_msgs::SetMode setMode;
	mavros_msgs::State currentState;
	ros::ServiceClient setModeClient = node.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	setMode.request.custom_mode = mode;

	if( currentState.mode != mode )
	{
		if( setModeClient.call(setMode) )
			ROS_INFO_STREAM("MODE UPDATED TO: " << mode);
		else
			ROS_ERROR_STREAM("FAILED TO UPDATE MODE TO: " << mode);
	}
}

