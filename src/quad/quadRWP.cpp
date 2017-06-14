#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <csignal>
#include <boost/thread.hpp>
#include <ros/ros.h>
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

bool waypointPusher( mavros_msgs::WaypointPush &pusher, ros::ServiceClient client, ros::NodeHandle node, 
	int frame, int command, bool isCurrent, bool autoCont, 
	float param1, float param2, float param3, float param4, 
	float lat, float lon, float alt );

float randCoord( float high, float low );

int main(int argc, char** argv)
{
	srand(static_cast <unsigned> (time(0)));
	ros::init(argc, argv, "mavros_mission_push");

	ros::Time::init();

	int rate = 10;
	ros::Rate r(rate);

	ros::NodeHandle n;
	ros::ServiceClient pushClient;
	mavros_msgs::WaypointPush wayPusher;

	float vertexNEx = 39.539152;
	float vertexNEy = -119.814124;
	float vertexSWx = 39.537778;
	float vertexSWy = -119.814219;

	int totalTime;
	int inputSpeed;
	int pause;
/*
	ROS_INFO("Enter x coordinate for NE vertex: ");
	std::cin >> vertexNEx;
	ROS_INFO("Enter y coordinate for NE vertex: ");
	std::cin >> vertexNEy;
	ROS_INFO("Enter x coordinate for SW vertex: ");
	std::cin >> vertexSWx;
	ROS_INFO("Enter y coordinate for SW vertex: ");
	std::cin >> vertexSWy;
*/
	ROS_INFO("Enter time to run: ");
	std::cin >> totalTime;

	ROS_INFO("Enter desired speed (default is 16): ");
	std::cin >> inputSpeed;

	ROS_INFO("Enter the pause time ");
	std::cin >> pause;

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
	maxThrottleVal.integer = inputSpeed;
	maxThrottleSetter.request.value = maxThrottleVal;
	maxThrottleSetter.request.param_id = "THR_MAX";

	mavros_msgs::ParamValue throttleVal;
	mavros_msgs::ParamSet throttleSetter;

	throttleVal.real = 0.0;
	throttleVal.integer = inputSpeed;
	throttleSetter.request.value = throttleVal;
	throttleSetter.request.param_id = "CRUISE_THROTTLE";

	ros::ServiceClient speedClient;
	ros::ServiceClient speedPushClient;
	mavros_msgs::ParamValue speedVal;
	mavros_msgs::ParamSet speedSetter;
	mavros_msgs::ParamPush speedPusher;

	speedClient = n.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
	speedPushClient = n.serviceClient<mavros_msgs::ParamPush>("mavros/param/push");
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

	
	//Activate auto mode
	mavros_msgs::SetMode setModeAuto;
	mavros_msgs::State currentState;
	ros::ServiceClient setModeClient = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	setModeAuto.request.custom_mode = "AUTO";

	if( currentState.mode != "AUTO" )
	{
		if( setModeClient.call(setModeAuto) )
			ROS_INFO("AUTO MODE ENABLED");
		else
			ROS_ERROR("FAILED TO ENABLE AUTO MODE");
	}


	float currentLat;
	float currentLong;

	time_t endwait;
	endwait = time(NULL) + totalTime;

	time_t pausewait;
	currentLat = randCoord( vertexNEy, vertexSWy );
	currentLong = randCoord( vertexNEx, vertexSWx );
	std::cout << "Pushed coords: " << currentLong << " , " << currentLat << std::endl;
	waypointPusher( wayPusher, pushClient, n, 2, 16, true, true, 0, 0, 0, 0, currentLat, currentLong, 50);

	ros::ServiceClient waypointClient = n.serviceClient<mavros_msgs::WaypointPull>("mavros/mission/pull");

	mavros_msgs::WaypointPull::Request req;
	mavros_msgs::WaypointPull::Response resp;
	bool clientCallSuccess;
	//while true loop
	while(time(NULL) < endwait)
	{
		clientCallSuccess = waypointClient.call(req, resp);	
		//if wp reached, push new random wp
		if(clientCallSuccess && resp.wp_received < 2 )
		{
			maxThrottleVal.integer = 0;
			maxThrottleSetter.request.value = maxThrottleVal;
			maxThrottleSetter.request.param_id = "THR_MAX";
			speedSetSucc = speedClient.call(maxThrottleSetter);

			pausewait = time(NULL) + pause;
			if(!speedSetSucc)
				ROS_WARN("WARNING: FAILED TO SET MAX THROTTLE TO 0");
			while(time(NULL) < pausewait)
			{
				ROS_INFO("PAUSING...");
			}
			maxThrottleVal.integer = inputSpeed;
			maxThrottleSetter.request.value = maxThrottleVal;
			maxThrottleSetter.request.param_id = "THR_MAX";
			speedSetSucc = speedClient.call(maxThrottleSetter);
			if(!speedSetSucc)
				ROS_WARN("WARNING: FAILED TO RESET MAX THROTTLE");

			currentLat = randCoord( vertexNEy, vertexSWy );
			currentLong = randCoord( vertexNEx, vertexSWx );
			std::cout << "Pushed coords: " << currentLong << " , " << currentLat << std::endl;
			waypointPusher( wayPusher, pushClient, n, 2, 16, true, true, 0, 0, 0, 0, currentLat, currentLong, 50);
		}
		ros::spinOnce();
		r.sleep();
		std::cout << time(NULL) << std::endl;
	}

	//Activate manual mode
	setModeAuto.request.custom_mode = "MANUAL";

	if( currentState.mode != "MANUAL" )
	{
		if( setModeClient.call(setModeAuto) )
			ROS_INFO("MANUAL MODE ENABLED");
		else
			ROS_ERROR("FAILED TO ENABLE MANUAL MODE");
	}

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

float randCoord( float high, float low )
{
	float randomNum = high + static_cast <float> (rand()) / ( static_cast <float> (RAND_MAX/(high-low)));
	return randomNum;
}
