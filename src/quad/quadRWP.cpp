#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <csignal>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamPush.h>
#include <mavros_msgs/ParamGet.h>

//Global latitude and longitude
double botLon;
double botLat;

bool waypointPusher( mavros_msgs::WaypointPush &pusher, ros::ServiceClient client, ros::NodeHandle node, 
	int frame, int command, bool isCurrent, bool autoCont, 
	float param1, float param2, float param3, float param4, 
	float lat, float lon, float alt );

double randCoord( double high, double low );

bool erleInit( int speed, ros::NodeHandle &node );

void setBotMode( std::string mode, ros::NodeHandle &node );

int getWaypointAmt( ros::NodeHandle &node );

void getBotCoords( const sensor_msgs::NavSatFix& msg );

int main( int argc, char** argv )
{
	//Initial ROS settings
	srand(static_cast <unsigned> (time(0)));
	ros::init(argc, argv, "mavros_mission_push");
	ros::Time::init();
	int rate = 10;
	ros::Rate r(rate);
	ros::NodeHandle n;

	//Objects to send waypoints to bot
	ros::ServiceClient pushClient;
	mavros_msgs::WaypointPush wayPusher;

	//Obtain data from user
	double vertexNEx = 39.539152;
	double vertexNEy = -119.814124;
	double vertexSWx = 39.537778;
	double vertexSWy = -119.814219;

	int totalTime = 10;
	int inputSpeed = 16;
	int pause = 3;
/*
	ROS_INFO("Enter x coordinate for NE vertex: ");
	std::cin >> vertexNEx;
	ROS_INFO("Enter y coordinate for NE vertex: ");
	std::cin >> vertexNEy;
	ROS_INFO("Enter x coordinate for SW vertex: ");
	std::cin >> vertexSWx;
	ROS_INFO("Enter y coordinate for SW vertex: ");
	std::cin >> vertexSWy;
	ROS_INFO("Enter time to run: ");
	std::cin >> totalTime;
	ROS_INFO("Enter desired speed (default is 16): ");
	std::cin >> inputSpeed;
	ROS_INFO("Enter the pause time ");
	std::cin >> pause;
*/
	//Initialize MAVROS parameters
	if( erleInit(inputSpeed, n) )
		ROS_INFO("FINISHED INITIALIZING PARAMETERS");

	setBotMode("AUTO", n);

	//Set up initial waypoint
	double currentLat;
	double currentLong;
	currentLat = randCoord( vertexNEy, vertexSWy );
	currentLong = randCoord( vertexNEx, vertexSWx );
	std::cout << "Pushed coords: " << currentLong << " , " << currentLat << std::endl;
	waypointPusher( wayPusher, pushClient, n, 2, 22, true, true, 15, 0, 0, 0, (float)currentLat, (float)currentLong, 50);

	time_t endwait;
	time_t pausewait;
	endwait = time(NULL) + totalTime;
	
	ros::Subscriber gpsSub1 = n.subscribe("/mavros/global_position/global", 1000, &getBotCoords);

	//while true loop
	while(time(NULL) < endwait)
	{
		std::cout << "botLat: " << botLat << " , botLon: " << botLon << std::endl;
		//if wp reached, push new random wp
		if(botLat == currentLat && botLon == currentLong)
		{
			erleInit(0, n);
			ros::spinOnce();
			pausewait = time(NULL) + pause;
			ROS_INFO("PAUSING...");
			while(time(NULL) < pausewait)
			{
				//do nothing
			}
			erleInit(16, n);
			ros::spinOnce();
			currentLat = randCoord( vertexNEy, vertexSWy );
			currentLong = randCoord( vertexNEx, vertexSWx );
			std::cout << "Pushed coords: " << std::setprecision(9) << currentLong << " , " << std::setprecision(9) << currentLat << std::endl;
			waypointPusher( wayPusher, pushClient, n, 3, 16, true, true, 0, 0, 0, 0, (float)currentLat, (float)currentLong, 50);
		}
		ros::spinOnce();
		r.sleep();
	}

	setBotMode("MANUAL", n);

	return 0;
}

bool waypointPusher( mavros_msgs::WaypointPush &pusher, ros::ServiceClient client, ros::NodeHandle node, 
	int frame, int command, bool isCurrent, bool autoCont, 
	float param1, float param2, float param3, float param4, 
	float lat, float lon, float alt )
{
	client = node.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
	
	mavros_msgs::Waypoint nextWaypoint;

	nextWaypoint.frame = frame;
	nextWaypoint.command = command;
	nextWaypoint.is_current = isCurrent;
	nextWaypoint.autocontinue = autoCont;
	nextWaypoint.param1 = param1;
	nextWaypoint.param2 = param2;
	nextWaypoint.param3 = param3;
	nextWaypoint.param4 = param4;
	nextWaypoint.x_lat = lat;
	nextWaypoint.y_long = lon;
	nextWaypoint.z_alt = alt;

	pusher.request.waypoints.push_back(nextWaypoint);

	if( client.call( pusher) )
		ROS_INFO("PUSHED WAYPOINT");
	else
		ROS_ERROR("PUSH FAILED");

	return client.call(pusher);
}

double randCoord( double high, double low )
{
	double randomNum = high + static_cast <double> (rand()) / ( static_cast <double> (RAND_MAX/(high-low)));
	return randomNum;
}

bool erleInit(int speed, ros::NodeHandle &node)
{
	//Update speed using param
	mavros_msgs::ParamValue steerVal;
	mavros_msgs::ParamSet steerSetter;
	steerVal.real = 2.0;
	steerVal.integer = 0;
	steerSetter.request.value = steerVal;
	steerSetter.request.param_id = "STEER2SRV_P";

	mavros_msgs::ParamValue turnMaxGVal;
	mavros_msgs::ParamSet turnMaxGSetter;
	turnMaxGVal.real = 1.0;
	turnMaxGVal.integer = 0;
	turnMaxGSetter.request.value = turnMaxGVal;
	turnMaxGSetter.request.param_id = "TURN_MAX_G";

	mavros_msgs::ParamValue navAggVal;
	mavros_msgs::ParamSet navAggSetter;
	navAggVal.real = 0.0;
	navAggVal.integer = 6;
	navAggSetter.request.value = navAggVal;
	navAggSetter.request.param_id = "NAVL1_PERIOD";

	mavros_msgs::ParamValue turnGainVal;
	mavros_msgs::ParamSet turnGainSetter;
	turnGainVal.real = 0.0;
	turnGainVal.integer = 100;
	turnGainSetter.request.value = turnGainVal;
	turnGainSetter.request.param_id = "SPEED_TURN_GAIN";

	mavros_msgs::ParamValue maxThrottleVal;
	mavros_msgs::ParamSet maxThrottleSetter;
	maxThrottleVal.real = 0.0;
	maxThrottleVal.integer = speed;
	maxThrottleSetter.request.value = maxThrottleVal;
	maxThrottleSetter.request.param_id = "THR_MAX";

	mavros_msgs::ParamValue throttleVal;
	mavros_msgs::ParamSet throttleSetter;
	throttleVal.real = 0.0;
	throttleVal.integer = speed;
	throttleSetter.request.value = throttleVal;
	throttleSetter.request.param_id = "CRUISE_THROTTLE";

	ros::ServiceClient speedClient;
	ros::ServiceClient speedPushClient;
	mavros_msgs::ParamValue speedVal;
	mavros_msgs::ParamSet speedSetter;
	mavros_msgs::ParamPush speedPusher;

	speedClient = node.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
	speedPushClient = node.serviceClient<mavros_msgs::ParamPush>("mavros/param/push");
	speedVal.real = 2.0;
	speedVal.integer = 0;
	speedSetter.request.value = speedVal;
	speedSetter.request.param_id = "CRUISE_SPEED";

	bool speedSetSucc = speedClient.call(speedSetter);
	speedSetSucc = speedClient.call(throttleSetter);
	speedSetSucc = speedClient.call(maxThrottleSetter);
	speedSetSucc = speedClient.call(steerSetter);
	speedSetSucc = speedClient.call(turnMaxGSetter);
	speedSetSucc = speedClient.call(navAggSetter);
	speedSetSucc = speedClient.call(turnGainSetter);

	bool speedPushSucc = speedPushClient.call(speedPusher);


	if( speedSetSucc && speedPushSucc )
		ROS_INFO("SPEED SUCCESSFULLY UPDATED");
	else
		ROS_INFO("SPEED UPDATE FAILED");

	return speedPushSucc;
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

int getWaypointAmt( ros::NodeHandle &node )
{
	ros::ServiceClient waypointClient = node.serviceClient<mavros_msgs::WaypointPull>("mavros/mission/pull");
	mavros_msgs::WaypointPull::Request req;
	mavros_msgs::WaypointPull::Response resp;
	bool clientCallSuccess;


	clientCallSuccess = waypointClient.call(req, resp);

	if( clientCallSuccess )
		ROS_INFO("WAYPOINT PULL SUCCESSFUL");
	else
		ROS_INFO("WAYPOINT PULL FAILED");

	return resp.wp_received;
}

void getBotCoords( const sensor_msgs::NavSatFix& msg )
{
	botLon = msg.longitude;
	botLat = msg.latitude;
}
