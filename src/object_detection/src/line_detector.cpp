//
// Created by dan on 6/05/18.
//
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv_apps/LineArrayStamped.h>
#include <opencv_apps/CircleArrayStamped.h>

class LineDetector {
public:
    LineDetector() {
        // Publish the scan information and  condensed lines to the image topic and line topic.
        image_pub = handle.advertise<sensor_msgs::Image>("/image", 1);
        lines_pub = handle.advertise<opencv_apps::LineArrayStamped>("/object_detection/lines", 1);

        // Subscribe to the scan and line topics.
        scan_sub = handle.subscribe("/scan", 1, &LineDetector::laserScanCallback, this);
        lines_sub = handle.subscribe("/hough_lines/lines", 1, &LineDetector::houghLinesCallback, this);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laserScanData) {
        // Convert the laser scan data into a two dimensional binary array representing the objects that have been detected.


        // Max angle and min angle of the laser scanner divide by the increment angle of each data point.
        int rangeDataNum =
                1 + int((laserScanData->angle_max - laserScanData->angle_min) / (laserScanData->angle_increment));

        int x = 0;
        int y = 0;
        unsigned int multiplier = 100;
        unsigned int width = 12 * multiplier;
        unsigned int height = 6 * multiplier;

        // Create image array and set content to zero.
        uint8_t img[height * width];
        memset(img, 0, sizeof(img));
        // Loop through laser data and store the x and y co-ordinates in an array.
        for (int j = 0; j < rangeDataNum; j++) {
            // Only take laser information that is less than 5m from us.
            if (laserScanData->ranges[j] < 5) {
                // Uses x = r * cos(theta) + 6 (to center the x axis) * multiplier
                x = int((laserScanData->ranges[j] * cos(j * laserScanData->angle_increment) + 6) * multiplier);
                y = int(laserScanData->ranges[j] * sin(j * laserScanData->angle_increment) * multiplier);

                ROS_DEBUG("x co-ordinate found: %d", x);
                ROS_DEBUG("y co-ordinate found: %d", y);

                ROS_DEBUG("Inserting point at %d", width * y + x);
                img[width * y + x] = 255;

                // Colour in the points around the current point to improve detection.
                if ((y > 1 && x > 1) && (x < width && y < height)) {
                    img[width * y + x - 1] = 255;
                    img[width * y + x + 1] = 255;
                    img[width * (y + 1) + x] = 255;
                    img[width * (y - 1) + x] = 255;
                }
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
        for (int i = 0; i < (width * height); i++) {
            image.data.push_back(img[i]);
        }

        ROS_DEBUG("Publishing Image!");
        image_pub.publish(image);
    }

    void houghLinesCallback(const opencv_apps::LineArrayStampedConstPtr &lineData) {
        // Lines that are next to each other need to be collated to find at least two sides of the object.
        std::vector<opencv_apps::Line> lines;
        opencv_apps::LineArrayStamped linesMsg;

        int d1x12x1Diff;
        int d1y12y1Diff;
        int d1x22x2Diff;
        int d1y22y2Diff;
        int thres = 10;

        for (int i = 0; i < lineData->lines.size(); i++) {
            bool different = true;
            opencv_apps::Line line = lineData->lines[i];
            for (int j = 0; j < lines.size(); j++) {
                // The delta difference is currentLine_x/lines_x + currentLine_y/lines_y
                // The closer this value is to 2, the closer these two points are to being equal to each other.
                d1x12x1Diff = abs(int(line.pt1.x - lines[j].pt1.x));
                d1y12y1Diff = abs(int(line.pt1.y - lines[j].pt1.y));
                d1x22x2Diff = abs(int(line.pt2.x - lines[j].pt2.x));
                d1y22y2Diff = abs(int(line.pt2.y - lines[j].pt2.y));

                ROS_DEBUG("%d %d %d %d", d1x12x1Diff, d1y12y1Diff, d1x22x2Diff, d1y22y2Diff);

                if (d1x12x1Diff < thres && d1y12y1Diff < thres && d1x22x2Diff < thres && d1y22y2Diff < thres) {
                    // We have a new line that is the same as the current one.
                    different = false;
                }
            }
            if (different) {
                lines.push_back(line);
                linesMsg.lines.push_back(line);
                ROS_DEBUG("Line found x1,y1: %.4f,%.4f x2,y2: %.4f,%.4f", line.pt1.x, line.pt1.y, line.pt2.x,
                          line.pt2.y);
            }
        }

        lines_pub.publish(linesMsg);
    }

private:
    ros::NodeHandle handle;

    ros::Publisher image_pub;
    ros::Publisher lines_pub;
    ros::Subscriber scan_sub;
    ros::Subscriber lines_sub;
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
