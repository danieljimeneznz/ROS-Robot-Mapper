//
// Created by dan on 6/05/18.
//
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv_apps/LineArrayStamped.h>
#include <opencv_apps/ContourArrayStamped.h>

class LineDetector {
public:
    LineDetector() {
        // Publish the scan information to the image topic.
        image_pub = handle.advertise<sensor_msgs::Image>("/image", 1);

        // Subscribe to the scan, line and contour topics.
        scan_sub = handle.subscribe("/scan", 1, &LineDetector::laserScanCallback, this);
//        lines_sub = handle.subscribe("/hough_lines/lines", 1, &LineDetector::houghLinesCallback, this);
        contour_sub = handle.subscribe("/find_contours/contours", 1, &LineDetector::houghContourCallback, this);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData) {
        // Convert the laser scan data into a two dimensional binary array representing the objects that have been detected.


        // Max angle and min angle of the laser scanner divide by the increment angle of each data point.
        int rangeDataNum = 1 + int((laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment));

        int x = 0;
        int y = 0;
        unsigned int multiplier = 100;
        unsigned int width = 12 * multiplier;
        unsigned int height = 6 * multiplier;

        // Create image array and set content to zero.
        uint8_t img[height * width];
        memset(img, 0, sizeof(img));
        // Loop through laser data and store the x and y co-ordinates in an array.
        for(int j = 0; j < rangeDataNum; j++) {
            // Only take laser information that is less than 5m from us.
            if (laserScanData->ranges[j] < 5) {
                // Uses x = r * cos(theta) + 6 (to center the x axis) * multiplier
                x = int((laserScanData->ranges[j] * cos(j * laserScanData->angle_increment) + 6) * multiplier);
                y = int(laserScanData->ranges[j] * sin(j * laserScanData->angle_increment) * multiplier);

                ROS_DEBUG("x co-ordinate found: %d", x);
                ROS_DEBUG("y co-ordinate found: %d", y);

                ROS_DEBUG("Inserting point at %d", width * y + x);
                img[width * y + x] = 255;
            }
        }

        // Set up the image message.
        sensor_msgs::Image image;
        image.height = height;
        image.width = width;
        image.encoding = "mono8";
        image.is_bigendian = 0;

        // Step is the full row length in bytes i.e. columns * number of channels * sizeof(datatype_used).
        image.step = width * sizeof(uint8_t);
        // Push the image array to the message.
        for(int i = 0; i < (width * height); i++) {
            image.data.push_back(img[i]);
        }

        ROS_DEBUG("Publishing Image!");
        image_pub.publish(image);
    }

//    void houghLinesCallback(const opencv_apps::LineArrayStampedConstPtr& lineData) {
//        ROS_DEBUG("Line Found from: %.4f to: %.4f", lineData->lines[0].pt1, lineData->lines[0].pt2);
//    }

    void houghContourCallback(const opencv_apps::ContourArrayStampedConstPtr& contourData) {

    }

private:
    ros::NodeHandle handle;

    ros::Publisher image_pub;
    ros::Subscriber scan_sub;
//    ros::Subscriber lines_sub;
    ros::Subscriber contour_sub;
};

int main(int argc, char **argv) {
    // Command line ROS arguments
    ros::init(argc, argv, "line_detector");


    // Loop 10 Hz
    //ros::Rate loop_rate(10);

    // Publish the image data to the image callback.
//    while (ros::ok()) {
//        ros::spinOnce();
//        loop_rate.sleep();
//
//        // Publish the image data.
//        image_pub.publish(image);
//    }

    LineDetector LineDetectorObject;

    ros::spin();

    return 0;
}
