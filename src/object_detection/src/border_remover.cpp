//
// Created by dan on 13/05/18.
//

#include <ros/ros.h>
#include <opencv_apps/LineArrayStamped.h>
#include <sensor_msgs/Image.h>
#include <tf/LinearMath/Vector3.h>

class BorderRemover {
public:

    BorderRemover() {
        image_pub = handle.advertise<sensor_msgs::Image>("/image", 1);
        lines_pub = handle.advertise<opencv_apps::LineArrayStamped>("/object/border", 1);

        // Subscribe to the lines topic.
        image_sub = handle.subscribe("/image/bordered", 1, &BorderRemover::ImageCallback, this);
        lines_sub = handle.subscribe("/lines", 1, &BorderRemover::houghLinesCallback, this);
    }

    void houghLinesCallback(const opencv_apps::LineArrayStampedConstPtr &lineData) {
        // See if two lines meet at a point and are perpendicular to each other.
        for (int i = 0; i < lineData->lines.size(); i++) {
            // Convert line points into vector.
            tf::Vector3 line1[2];
            line1[0] = tf::Vector3(lineData->lines[i].pt1.x, lineData->lines[i].pt1.y, 0);
            line1[1] = tf::Vector3(lineData->lines[i].pt2.x, lineData->lines[i].pt2.y, 0);

            for (int j = 0; j < lineData->lines.size(); j++) {
                if(i == j) {
                    continue;
                }

                // Convert line points into vector.
                tf::Vector3 line2[2];
                line2[0] = tf::Vector3(lineData->lines[j].pt1.x, lineData->lines[j].pt1.y, 0);
                line2[1] = tf::Vector3(lineData->lines[j].pt2.x, lineData->lines[j].pt2.y, 0);

                // See if they are perpendicular.
                tf::Vector3 line1pt = line1[0] - line1[1];
                tf::Vector3 line2pt = line2[0] - line2[1];
                double cross = fabs(tf::tfDot(line1pt, line2pt));

                // They are both perpendicular
                if (cross < 5.0) {
                    for (int k = 0; k < 2; k++) {
                        for (int l = 0; l < 2; l++) {
                            double distance = fabs(tf::tfDistance(line1[k], line2[l]));
                            // If distance is less than threshold then the two meet.
                            if (distance < 5.0) {
                                // We have two lines that are perpendicular and close to each other.
                                ROS_DEBUG("FOUND A BORDER CORNER");
                            }
                        }
                    }
                }
            }
        }

//        lines_pub.publish(line);
    }

    void ImageCallback(const sensor_msgs::ImageConstPtr &image) {
//        sensor_msgs::Image unBorderedImage;
//        unBorderedImage.width = image->width;
//        unBorderedImage.step = image->step;
//        unBorderedImage.is_bigendian = image->is_bigendian;
//        unBorderedImage.height = image->height;
//
//        unBorderedImage.data = image->data;
//        unBorderedImage.encoding = image->encoding;
        image_pub.publish(image);
    }

private:
    ros::NodeHandle handle;

    ros::Publisher image_pub;
    ros::Publisher lines_pub;
    ros::Subscriber lines_sub;
    ros::Subscriber image_sub;

    std::vector<opencv_apps::Line> borders;
};

int main(int argc, char **argv) {
    // Command line ROS arguments
    ros::init(argc, argv, "remove_borders");

    BorderRemover borderRemover;
    ros::spin();

    return 0;
}