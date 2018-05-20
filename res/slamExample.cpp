#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <string>
//gmapping required header files
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"

geometry_msgs::Twist velocityCommand; 
/*
The scan subscriber call back function
To understand the sensor_msgs::LaserScan object look at
http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
*/
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
// Compute the number of data points
// max angle and min angle of the laser scanner divide by the increment angle of each data point
float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);

//move forward
velocityCommand.linear.x = 0.1;
velocityCommand.angular.z = 0.0;
for(int j = 0; j < rangeDataNum; ++j) // Go through the laser data
{
if( laserScanData->ranges[j] < 0.5 )  // If there is an object within 0.5m
	{
		velocityCommand.linear.x = 0;   // stop forward movement
		velocityCommand.angular.z = 0.1; // turn left
		break;
	}
}}

int grid_x; 
int grid_y;
float map_o_x = 0;
float map_o_y = 0;
float map_r = 1;

/*
The OccupancyGrid is a 2-D grid map, in which 
each cell represents the probability of an object.
http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
*/
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) { 
	//get the origin of the map frame
	map_o_x = msg->info.origin.position.x;
	map_o_y = msg->info.origin.position.y;
	//get the resolution of each cell in the OccupancyGrid
	map_r = msg->info.resolution;
	
	std::string line;
	std::string output;
	
	//the occupancy grid is a 1D array representation of a 2-D grid map
	//compute the robot position in the 1D array
	int r_index = grid_x + grid_y * msg->info.width;
	
	int printEnable = 0;
	//go through the entire grid
	for (int i = 0; i < msg->info.width*msg->info.height; i++) {
		
		if (r_index == i) {
			//if the cell is the cell of the robot
			//print R
			line += "R";
			printEnable = 1;
		}else{
			//if the cell is unknown (-1) or empty (0) 
			//print a space
			if (msg->data[i] == -1) {
				line += " ";
			}else if (msg->data[i] == 0) {
				line += " ";
			}else{
				//if the cell may contain an object
				//print *
				line += "*";
				printEnable = 1;
			}
		}
		
		//at the start of each row
		if (i % msg->info.width == 0) {
			//add to the output if there is something interesting on this row
			if (printEnable == 1) {
				output = line + "\n" + output;
			}
			line = "";
			printEnable = 0;
		}
	}
	//print output
	printf("%s\n",output.c_str());
	
	//other things about the map that you can print
	/* 
	ROS_INFO("");
	ROS_INFO("X: [%d]", grid_x);
	ROS_INFO("Y: [%d]", grid_y);
	ROS_INFO("width: [%d]", msg->info.width);
	ROS_INFO("height: [%d]", msg->info.height);
	ROS_INFO("resolution: [%d]", msg->info.resolution);
	ROS_INFO("Map Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->info.origin.orientation.x, msg->info.origin.orientation.y, msg->info.origin.orientation.z, msg->info.origin.orientation.w);
	*/
}

int main (int argc, char **argv)
{	
	ros::init(argc, argv, "pioneer_laser_node");	// command line ROS arguments
	ros::NodeHandle my_handle;	// ROS comms access point

	ros::Publisher vel_pub_object = my_handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);

	/*
	subscribe to the scan topic and define a callback function to process the data
	the call back function is called each time a new data is received from the topic
	*/
	ros::Subscriber laser_sub_object = my_handle.subscribe("/scan", 1, laserScanCallback);

	/*
	subscribe to the map created by gmapping
	*/
	ros::Subscriber sub_map = my_handle.subscribe("map", 1, mapCallback);
	ros::Rate loop_rate(10);// loop 10 Hz

	tf::TransformListener listener;
	while(ros::ok()) // publish the velocity set in the call back
	{
		ros::spinOnce();
		loop_rate.sleep();
		
		tf::StampedTransform transform;
		try{
			//transform the coordinate frame of the robot to that of the map
			//(x,y) index of the 2D Grid
			listener.lookupTransform("map", "base_link",ros::Time(0), transform);
			grid_x = (unsigned int)((transform.getOrigin().x() - map_o_x) / map_r);
			grid_y = (unsigned int)((transform.getOrigin().y() - map_o_y) / map_r);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s\n",ex.what());
			ros::Duration(1.0).sleep();
		}
		
		// publish to the twist to the topic
		vel_pub_object.publish(velocityCommand);
	}

	return 0;
}