//
// Created by dan on 13/05/18.
//

#include <ros/ros.h>
#include <opencv_apps/LineArrayStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <tf/LinearMath/Vector3.h>
#include <object_detection/object_detection.h>
#include <geometry_msgs/Vector3.h>

class BorderRemover {
public:

    BorderRemover() {
        image_pub = handle.advertise<sensor_msgs::Image>("/image", 1);
        border_pub = handle.advertise<geometry_msgs::Vector3>("/object/border", 1);

        // Subscribe to the lines topic.
        image_sub = handle.subscribe("/image/bordered", 1, &BorderRemover::ImageCallback, this);
        lines_sub = handle.subscribe("/lines", 1, &BorderRemover::houghLinesCallback, this);
    }

    void houghLinesCallback(const opencv_apps::LineArrayStampedConstPtr &lineData) {
        this->lines = *lineData;

        tf::Vector3 borderCorner;

        // See if two lines meet at a point and are perpendicular to each other.
        for (int i = 0; i < lineData->lines.size(); i++) {
            for (int j = 0; j < lineData->lines.size(); j++) {
               borderCorner = object_detection::linesArePerpendicular(lineData->lines[i], lineData->lines[j]);

                if (borderCorner.length() != 0.0) {
                    geometry_msgs::Vector3 borderMsg;
                    borderMsg.x = borderCorner.x();
                    borderMsg.y = borderCorner.y();
                    borderMsg.z = borderCorner.z();
                    border_pub.publish(borderMsg);
                }
            }
        }
    }

    void ImageCallback(const sensor_msgs::ImageConstPtr &image) {
        // Points along lines can be immediately removed from the image.
        // The way this is done by computing the area between points on the image and then removing them if
        // the area is above a certain threshold.

//        cv_bridge::CvImageConstPtr cv_ptr;
//
//        try {
//            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
//        } catch (cv_bridge::Exception& e) {
//            ROS_ERROR("cv_bridge exception: %s", e.what());
//            return;
//        }
//
//        cv::Mat mat = cv_ptr->image;
//
//        for (int i = 0; i < lines.lines.size(); i++) {
//            tf::Vector3 a = tf::Vector3(lines.lines[i].pt1.x, lines.lines[i].pt1.y, 0);
//            tf::Vector3 b = tf::Vector3(lines.lines[i].pt2.x, lines.lines[i].pt2.y, 0);
//            tf::Vector3 ab = a - b;
//            tf::Vector3 ac;
//            double area;
//
//            // Set line endpoints equal to zero.
//            mat.at<int>(int(lines.lines[i].pt1.y), int(lines.lines[i].pt1.x)) = 0;
//            mat.at<int>(int(lines.lines[i].pt2.y), int(lines.lines[i].pt2.x)) = 0;
//
//            for (int y = 0; y < mat.rows; y++) {
//                for (int x = 0; x < mat.cols; x++) {
//                    // Point at current position is white.
//                    if (mat.at<int>(y, x) == 255) {
//                        ac = a - tf::Vector3(y, x, 0);
//                        area = 0.5 * tf::tfCross(ab, ac).length();
//
//                        if(area < 100000.0) {
//                            mat.at<int>(y, x) = 0;
//                        }
//                    }
//                }
//            }
//        }
//
//        image_pub.publish(cv_ptr->toImageMsg());
        image_pub.publish(image);
    }

private:
    ros::NodeHandle handle;

    ros::Publisher image_pub;
    ros::Publisher border_pub;
    ros::Subscriber lines_sub;
    ros::Subscriber image_sub;

    opencv_apps::LineArrayStamped lines;
};

int main(int argc, char **argv) {
    // Command line ROS arguments
    ros::init(argc, argv, "remove_borders");

    BorderRemover borderRemover;
    ros::spin();

    return 0;
}