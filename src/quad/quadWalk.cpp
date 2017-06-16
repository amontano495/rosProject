#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <csignal>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamPush.h>
#include <mavros_msgs/ParamGet.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

//Local Quarternion Values
geometry_msgs::Quaternion botQ;

bool erleInit(int speed, ros::NodeHandle &node );

void setBotMode( std::string mode, ros::NodeHandle &node );

void getBotQ( const sensor_msgs::Imu &msg );

void getCmpHdg( const std_msgs::Float64 &msg );

int main(int argc, char** argv)
{
	//Initial ROS settings
	ros::init(argc, argv, "erle_rand_walker");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/mavros/imu/data", 1000, &getBotQ);
	double roll, pitch, yaw;

	while(ros::ok())
	{
		yaw = tf::getYaw( botQ );	
		std::cout << " Yaw: " << yaw << std::endl;
		ros::spinOnce();
	}

	return 0;
}

void getCmpHdg( const std_msgs::Float64 &msg )
{
	ROS_INFO("SUBSCRIBER INVOKED");
}

void getBotQ( const sensor_msgs::Imu &msg )
{
	std::cout << "X: " << msg.orientation.x << ", Y: " << msg.orientation.y << ", Z: " << msg.orientation.z << ", W: " << msg.orientation.w << std::endl;
	botQ = msg.orientation;

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

