//
// Created by dan on 13/05/18.
//
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class ScanCleaner {
public:

    /**
     * Create a new ScanCleaner instance.
     */
    ScanCleaner() {
        // Publish the clean scan information to the /object_detection/scan topic
        scan_pub = handle.advertise<sensor_msgs::LaserScan>("/object_detection/scan", 1);

        // Subscribe to the scan topic.
        scan_sub = handle.subscribe("/scan", 1, &ScanCleaner::laserScanCallback, this);

        threshold = 0.1;
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laserScanData) {
        // Max angle and min angle of the laser scanner divide by the increment angle of each data point.
        int rangeDataNum =
                1 + int((laserScanData->angle_max - laserScanData->angle_min) / (laserScanData->angle_increment));

        sensor_msgs::LaserScan cleanScanData;
        cleanScanData.angle_min = laserScanData->angle_min;
        cleanScanData.angle_max = laserScanData->angle_max;
        cleanScanData.angle_increment = laserScanData->angle_increment;
        cleanScanData.time_increment = laserScanData->time_increment;
        cleanScanData.scan_time = laserScanData->scan_time;
        cleanScanData.range_min = laserScanData->range_min;
        cleanScanData.range_max = laserScanData->range_max;
        cleanScanData.intensities = laserScanData->intensities;
        cleanScanData.header = laserScanData->header;

        for (int j = 0; j < rangeDataNum; j++) {
            // If a scan point is all by itself, remove that point from the scan. i.e. set the range at 5.7.
            // This cleans up the scan data noise.
            if (j > 0 && j < rangeDataNum - 1) {
                if(fabsf(laserScanData->ranges[j] - laserScanData->ranges[j - 1]) > threshold
                   && fabsf(laserScanData->ranges[j] - laserScanData->ranges[j + 1]) > threshold) {
                    cleanScanData.ranges.push_back(5.7f);
                    continue;
                }
            }
            cleanScanData.ranges.push_back(laserScanData->ranges[j]);
        }

        scan_pub.publish(cleanScanData);
    }

private:
    ros::NodeHandle handle;

    ros::Publisher scan_pub;
    ros::Subscriber scan_sub;

    float threshold;
};

int main(int argc, char **argv) {
    // Command line ROS arguments
    ros::init(argc, argv, "clean_scan");

    ScanCleaner scanCleaner;
    ros::spin();

    return 0;
}
