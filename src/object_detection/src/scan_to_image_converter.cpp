//
// Created by dan on 6/05/18.
//
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv_apps/CircleArrayStamped.h>

class ScanToImageConverter {
public:

    /**
     * Create a new LineDetector instance.
     */
    ScanToImageConverter() {
        // Publish the scan information and  condensed lines to the image topic and line topic.
        image_pub = handle.advertise<sensor_msgs::Image>("/image", 1);

        // Subscribe to the scan and line topics.
        scan_sub = handle.subscribe("/scan", 1, &ScanToImageConverter::laserScanCallback, this);
        multiplier = 100;
    }

    /**
     * Laser scan callback converts laser data into an image representing that objects that have been seen. This is
     * then fed to the /image topic for the hough_lines/hough_circles algorithm to find the lines/circles.
     *
     * @param laserScanData     The scan data.
     */
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laserScanData) {
        // Max angle and min angle of the laser scanner divide by the increment angle of each data point.
        int rangeDataNum =
                1 + int((laserScanData->angle_max - laserScanData->angle_min) / (laserScanData->angle_increment));

        int x = 0;
        int y = 0;
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

                ROS_DEBUG("Inserting point at %d", width * y + x);

                // X and Y must be positive and less than the width and height.
                // Colour in the points around the current point to improve detection.
                if ((y > 1 && x > 1) && (x < width && y < height)) {
                    img[width * y + x] = 255;

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

private:
    ros::NodeHandle handle;

    ros::Publisher image_pub;
    ros::Subscriber scan_sub;

    unsigned int multiplier;
};

int main(int argc, char **argv) {
    // Command line ROS arguments
    ros::init(argc, argv, "scan_to_image_converter");

    ScanToImageConverter lineDetector;
    ros::spin();

    return 0;
}
