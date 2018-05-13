//
// Created by dan on 13/05/18.
//

#include <ros/ros.h>
#include <opencv_apps/LineArrayStamped.h>
#include <sensor_msgs/Image.h>

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
            double line1[2][2];
            line1[0][0] = lineData->lines[i].pt1.x;
            line1[0][1] = lineData->lines[i].pt1.y;
            line1[1][0] = lineData->lines[i].pt2.x;
            line1[1][1] = lineData->lines[i].pt2.y;

            for (int j = 0; j < lineData->lines.size(); j++) {
                // Store the points in an array
                double line2[2][2];
                line2[0][0] = lineData->lines[j].pt1.x;
                line2[0][1] = lineData->lines[j].pt1.y;
                line2[1][0] = lineData->lines[j].pt2.x;
                line2[1][1] = lineData->lines[j].pt2.y;

                for(int k = 0; k < 2; k++) {
                    for(int l = 0; l < 2; l++) {
                        double distance = sqrt(pow(fabs(line1[k][0] - line2[l][0]),2) + pow(fabs(line1[k][1] - line2[l][1]),2));

                        // If distance is less than threshold then the two meet.
                        if (distance < 5.0) {
                            // See if they are perpendicular.
                            ROS_DEBUG("CLOSE but not perpendicular?");
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