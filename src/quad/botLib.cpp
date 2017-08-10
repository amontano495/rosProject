#ifndef BOTLIB_CPP
#define BOTLIB_CPP

#include "botLib.h"


//Global gps coords of the robot
double botLat;
double botLon;

//Global gps coords of the polygonal boundary
coord NWvert;
coord SWvert;
coord NEvert;
coord SEvert;
coord CenterVert;

//Sets the global coords of the robot to message returned from ROS
void getBotCoords( const sensor_msgs::NavSatFix &msg )
{
	botLat = msg.latitude;
	botLon = msg.longitude;
}

//Determines if global coords of bot within defined polygon
bool boundaryCheck( coord thePath[] )
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
		if( RayCrossesSegment( a, b ) )
			crossings++;
	}

	if( crossings % 2 == 1 )
	{
		botWithinBoundary = true;
	}

	else
	{
		botWithinBoundary = false;
		ROS_WARN_STREAM("OUT OF BOUNDS! BOT LAT: " << std::setprecision(10) << botLat << " BOT LON: " << std::setprecision(10) << botLon );
	}

	return botWithinBoundary;
}

bool RayCrossesSegment( coord a, coord b )
{
	double px = botLon;
	double py = botLat;
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

//Sets the bots speed and angle (values between 1100 - 1900)
void setBotMovement( int speed, int angle, ros::Publisher &rcPub )
{
	mavros_msgs::OverrideRCIn msg;

	msg.channels[0] = angle;
	msg.channels[2] = speed;
	rcPub.publish(msg);
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



int getUniRand( int min, int max )
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(min, max);

	return dis(gen);
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


void returnToBoundary( ros::NodeHandle &node )
{
	ros::ServiceClient pushClient;
	mavros_msgs::WaypointPush wayPusher;

	waypointPusher( wayPusher, pushClient, node, 2, 22, true, true, 15, 0, 0, 0, 39.538440, -119.814151, 50 );
	waypointPusher( wayPusher, pushClient, node, 3, 16, true, true, 0, 0, 0, 0, 39.538440, -119.814151, 50 );
}


void setPolyVerts()
{
	int vertCounter = 0;
	std::string vertUserInput;

	while( vertCounter < 4 )
	{
		std::cin >> vertUserInput;
		ros::spinOnce();
		if( vertUserInput == "NW" )
		{
			NWvert.lat = botLat;
			NWvert.lon = botLon;
			vertCounter++;
		}
		else if( vertUserInput == "NE" )
		{
			NEvert.lat = botLat;
			NEvert.lon = botLon;
			vertCounter++;
		}
		else if( vertUserInput == "SW" )
		{
			SWvert.lat = botLat;
			SWvert.lon = botLon;
			vertCounter++;
		}
		else if( vertUserInput == "SE" )
		{
			SEvert.lat = botLat;
			SEvert.lon = botLon;
			vertCounter++;
		}
	}
}

bool withinWaypointRadius( double lat, double lon )
{
	bool inCircle;
	double rSqrd = 0.00000002;
	double dSqrd = (pow((botLat - lat),2)) + (pow((botLon - lon),2));
	if( dSqrd <= rSqrd )
	{
		inCircle = true;
		ROS_WARN("WITHIN RADIUS");
	}
	else
		inCircle = false;

	return inCircle;
}
#endif
