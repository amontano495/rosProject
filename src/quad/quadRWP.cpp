#include <string>
#include <map>
#include <random>
#include <math.h>
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

#include "botLib.h"

coord randCoord( coord polygon[] );

bool erleInit( int speed, ros::NodeHandle &node );

int getWaypointAmt( ros::NodeHandle &node );

bool boundaryCheckRand( coord thePath[], double lat, double lon );

bool RayCrossesSegmentRand( coord a, coord b, double lat, double lon );

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

	ros::Subscriber gpsSub1 = n.subscribe("/mavros/global_position/raw/fix", 1000, &getBotCoords);

	//Obtain data from user

	coord quadPolygon[4];
	//NW vertex
	quadPolygon[0].lat = 39.539045;
	quadPolygon[0].lon = -119.814623;

	//SW vertex
	quadPolygon[1].lat = 39.537889;
	quadPolygon[1].lon = -119.813714;

	//NE vertex
	quadPolygon[2].lat = 39.539155;
	quadPolygon[2].lon = -119.814138;

	//SE vertex
	quadPolygon[3].lat = 39.537783;
	quadPolygon[3].lon = -119.814214;


	int totalTime = 120;
	int inputSpeed = 16;
	int pause = 3;

	//Initialize MAVROS parameters
	if( erleInit(inputSpeed, n) )
		ROS_INFO("FINISHED INITIALIZING PARAMETERS");


	setBotMode("MANUAL", n);
	//Set up initial waypoint
	double targetLat;
	double targetLong;
	coord targetCoord = randCoord( quadPolygon );
	targetLat = targetCoord.lat;
	targetLong = targetCoord.lon;

	std::cout << "Pushed coords: " << std::setprecision(9) << targetLat << " , " << std::setprecision(9) << targetLong << std::endl;
	waypointPusher( wayPusher, pushClient, n, 2, 22, true, true, 15, 0, 0, 0, targetLat, targetLong, 50 );
	waypointPusher( wayPusher, pushClient, n, 3, 16, true, true, 0, 0, 0, 0, targetLat, targetLong, 50 );

	setBotMode("AUTO", n);

	time_t endwait;
	time_t pausewait;
	endwait = time(NULL) + totalTime;


	//while true loop
	while(ros::ok())
	{
		//if wp reached, push new random wp
		if( withinWaypointRadius(targetLat, targetLong) )
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

			targetCoord = randCoord( quadPolygon );
			targetLat = targetCoord.lat;
			targetLong = targetCoord.lon;

			std::cout << "Pushed coords: " << std::setprecision(9) << targetLat << " , " << std::setprecision(9) << targetLong << std::endl;
			waypointPusher( wayPusher, pushClient, n, 3, 16, true, true, 0, 0, 0, 0, targetLat, targetLong, 50);
		}

		if( !(boundaryCheck(quadPolygon)) )
		{
			waypointClear( n );
			ROS_INFO("CLEARING WAYPOINTS..................");
			returnToBoundary( n );
			ROS_INFO("SET TO RETURN HOME..................");
			while( !(boundaryCheck(quadPolygon)) )
			{
				ros::spinOnce();
				r.sleep();
			}
			ROS_INFO("RETURNED");

			targetCoord = randCoord( quadPolygon );
			targetLat = targetCoord.lat;
			targetLong = targetCoord.lon;

			std::cout << "Pushed coords: " << std::setprecision(9) << targetLat << " , " << std::setprecision(9) << targetLong << std::endl;

			waypointPusher( wayPusher, pushClient, n, 2, 22, true, true, 15, 0, 0, 0, targetLat, targetLong, 50 );
			waypointPusher( wayPusher, pushClient, n, 3, 16, true, true, 0, 0, 0, 0, targetLat, targetLong, 50);
		}

		ros::spinOnce();
		r.sleep();
	}

	setBotMode("MANUAL", n);

	return 0;
}


coord randCoord( coord polygon[] )
{
	bool withinPolygon = false;
	double randLat;
	double randLon;
	coord randCoord;

	double latHigh;
	double lonHigh;
	double latLow;
	double lonLow;

	latHigh = polygon[2].lat;
	lonHigh = polygon[0].lon;
	latLow = polygon[3].lat;
	lonLow = polygon[1].lon;

	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<> distLat(latLow,latHigh);
	std::uniform_real_distribution<> distLon(lonHigh,lonLow);

	while( withinPolygon == false )
	{
		randLat = (double)distLat(e2);
		randLon = (double)distLon(e2);

		std::cout << "Rand Coords: " << std::setprecision(9) << randLat << " , " << std::setprecision(9) << randLon << std::endl;

		randCoord.lat = randLat;
		randCoord.lon = randLon;

		if( boundaryCheckRand( polygon, randLat, randLon ) )
			withinPolygon = true;
	}

	randCoord.lat = randLat;
	randCoord.lon = randLon;
	return randCoord;
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

bool boundaryCheckRand( coord thePath[], double lat, double lon )
{
	int crossings = 0;
	int count = 4;

	bool botWithinBoundary;

	int j;
	coord a, b;

	for( int i = 0; i < count; i++ )
	{
		a = thePath[i];
		j = i + 1;
		if( j >= count )
			j = 0;
		b = thePath[j];
		if( RayCrossesSegmentRand( a, b, lat, lon ) )
			crossings++;
	}

	if( crossings % 2 == 1 )
	{
		botWithinBoundary = true;
	}

	else
	{
		botWithinBoundary = false;
	}

	return botWithinBoundary;
}

bool RayCrossesSegmentRand( coord a, coord b, double lat, double lon )
{
	double px = lon;
	double py = lat;
	double ax = a.lon;
	double ay = a.lat;
	double bx = b.lon;
	double by = b.lat;
	if( ay > by )
	{
		ax = b.lon;
		ay = b.lat;
		bx = a.lon;
		by = b.lat;
	}

	if (px < 0) { px += 360; };
	if (ax < 0) { ax += 360; };
	if (bx < 0) { bx += 360; };

	if (py == ay || py == by) 
		py += 0.00000001;
	if ((py > by || py < ay) || (px > fmax(ax,bx)))
		return false;
	if (px < fmin(ax,bx))
		return true;

	double red = (ax != bx) ? ((by - ay) / (bx - ax)) : FLT_MAX;
	double blue = (ax != px) ? ((py - ay) / (px - ax)) : FLT_MAX;
	return (blue >= red);
}
