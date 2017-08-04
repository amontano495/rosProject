#include <string>
#include <vector>
#include <csignal>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/WaypointPush.h>
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mavros_mission_push");

	ros::Time::init();

	int rate = 10;
	ros::Rate r(rate);

	ros::NodeHandle n;
	ros::ServiceClient pushClient;
	mavros_msgs::WaypointPush wayPusher;

	//Send waypoints to rover
	waypointPusher( wayPusher, pushClient, n, 2, 22, true, true, 15, 0, 0, 0, 39.53915, -119.81411196, 50);

	waypointPusher( wayPusher, pushClient, n, 3, 16, true, true, 0, 0, 0, 0, 39.53786, -119.81372, 50);

	waypointPusher( wayPusher, pushClient, n, 3, 16, true, true, 0, 0, 0, 0, 39.53776847, -119.8142568, 50);

	waypointPusher( wayPusher, pushClient, n, 3, 16, true, true, 0, 0, 0, 0, 39.53913784, -119.81470204, 50);

	waypointPusher( wayPusher, pushClient, n, 3, 16, true, true, 0, 0, 0, 0, 39.539224709, -119.81424607, 50);


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
	maxThrottleVal.integer = 16;
	maxThrottleSetter.request.value = maxThrottleVal;
	maxThrottleSetter.request.param_id = "THR_MAX";

	mavros_msgs::ParamValue throttleVal;
	mavros_msgs::ParamSet throttleSetter;

	throttleVal.real = 0.0;
	throttleVal.integer = 16;
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

	//while true loop
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
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
		ROS_INFO_STREAM("PUSHED WAYPOINT");
	else
		ROS_INFO_STREAM("PUSH FAILED");

	return client.call(pusher);
}
