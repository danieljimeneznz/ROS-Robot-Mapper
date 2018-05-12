#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Twist velocityCommand; 
int test = 0;

/*
The scan subscriber call back function
To understand the sensor_msgs::LaserScan object look at
http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
*/
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData) {
  // Compute the number of data points
  // max angle and min angle of the laser scanner divide by the increment angle of each data point
  int rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);
  
  // move forward
  //velocityCommand.linear.x = 0.1;
  //velocityCommand.angular.z = 0.0;

  if (test != 0) {
    return;
  }
  test++;
  
  // Go through laser data.
  for(int j = 0; j < rangeDataNum; j++) {
    ROS_INFO("%.4f", laserScanData->ranges[j]);
    // If there is an object within 0.5m
    //if( laserScanData->ranges[j] < 0.5 ) {
      //velocityCommand.linear.x = 0;   // stop forward movement
      //velocityCommand.angular.z = 0.1; // turn left
      //break;
    //}
  }
}

//void odomCallback(const nav_msgs::Odometry::ConstPtr& odometryData) {
//}

int main (int argc, char **argv) {	
  ros::init(argc, argv, "pioneer_laser_node");	// command line ROS arguments
  ros::NodeHandle my_handle;	// ROS comms access point
  
  ros::Publisher vel_pub_object = my_handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);
  
  /*
   subscribe to the scan topic and define a callback function to process the data
   the call back function is called each time a new data is received from the topic
  */
  ros::Subscriber laser_sub_object = my_handle.subscribe("/scan", 1, laserScanCallback);
  //ros::Subscriber nav_sub = my_handle.subscribe("/odom", 1, odomCallback);
  
  ros::Rate loop_rate(10);// loop 10 Hz
  
  // publish the velocity set in the call back
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();

    // publish to the twist to the topic
//    vel_pub_object.publish(velocityCommand);
  }
  
  return 0;
}
