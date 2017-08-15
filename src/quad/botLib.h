#ifndef BOTLIB_H
#define BOTLIB_H

#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointClear.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/WaypointPush.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>


//data type to represent lattitude and longitude
struct coord {
	double lat;
	double lon;
} ;

//Sets the global coords to what the GPS device responds
void getBotCoords( const sensor_msgs::NavSatFix& msg );

//Determines if the bot is within the given two vertices
//The two vertices represent opposite corners of a given square
bool boundaryCheck( coord thePath[] );

//Used by boundary check
bool RayCrossesSegment( coord a, coord b );

//Sets the current mode of the robot (AUTO, MANUAL, GUIDED, etc)
void setBotMode( std::string mode, ros::NodeHandle &node );

//Returns a uniformly distributed random number each time it is called
int getUniRand( int min, int max );

//pushes waypoints to the rover
bool waypointPusher( mavros_msgs::WaypointPush &pusher, ros::ServiceClient client, ros::NodeHandle node, 
	int frame, int command, bool isCurrent, bool autoCont, 
	float param1, float param2, float param3, float param4, 
	float lat, float lon, float alt );

//Sends the rover back to within the boundaries
void returnToBoundary( ros::NodeHandle &node );

//Captures the vertices of the polygon
void setPolyVerts( ros::NodeHandle &node, coord &NW, coord &NE, coord &SW, coord &SE);

//Captures the vertices of the polygon
void setPolyVertsFromFile( coord &NW, coord &NE, coord &SW, coord &SE);

//Updates the speed of the bot
void setBotMovement( int speed, int angle, ros::Publisher &rcPub );

//Checks if the rover is within a defined radius where params are center
bool withinWaypointRadius( double lat, double lon );

//Clears the waypoint mission
bool waypointClear( ros::NodeHandle node );

#endif
