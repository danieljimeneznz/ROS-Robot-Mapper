//
// Created by dan on 7/05/18.
//
#include <ros/ros.h>
#include <opencv_apps/LineArrayStamped.h>
#include <opencv_apps/CircleArrayStamped.h>

class Object {
public:
    std::string type;
    double x;
    double y;
};

class Barrel : Object {
public:
    double radius;
};

class Container : Object {
public:
    double width;
    double height;
};

class ObjectDetector {
public:
    ObjectDetector() {
        // Subscribe to the line and circle topics.
        lines_sub = handle.subscribe("/object_detection/lines", 1, &ObjectDetector::linesCallback, this);
        circle_sub = handle.subscribe("/hough_circles/circles", 1, &ObjectDetector::circlesCallback, this);
    }

    void linesCallback(const opencv_apps::LineArrayStampedConstPtr &lineData) {
        // Convert lines into a box centre co-ordinate if the lines are almost perpendicular to each other.
        // if the lines are too long then assume walls. i.e. > 1m or 100px.
        // Get centre co-ordinate referenced to the global frame.
        // add object to bin.

        // container = centre, length and width.
    }

    void circlesCallback(const opencv_apps::CircleArrayStampedConstPtr &circleData) {
        // Get the centre co-ordinate referenced to the global frame.
        // add object to bin.
        // barrel = centre, radius.
    }

    void addObjectToBin(const Object &object) {
        // Place in the bin if not already in there.
        // Increment bin counter.
    }

    void findObjects() {
        // Consolidate objects from the bin.
        // Empty the bin
        // Add the found object to the object map if it does not already exist.
        // Display found object to console.
    }

private:
    ros::NodeHandle handle;

    ros::Subscriber lines_sub;
    ros::Subscriber circle_sub;

    // x and y coordinates relative to the global reference frame.
    // Objects stored as map i.e. { Id: Object }
    std::map<int, Object> objects;

    // Object Bin represents all the possible objects that could exist in the world at that point (box or circle).
    std::vector<Object> objectBin;
    // Object Bin count is a counter that counts how many possible times the object has been detected as a box or circle.
    std::vector<unsigned int> objectBinCount;
    unsigned int threshold = 100;

    // e.g. a box is detected at x = 10, y = 5, 120 times.
    // a circle is detected at x = 10, y = 6, 25 times.
    // Therefore the found object is a box as it has been detected at a point over the threshold of 100 times.

};

int main(int argc, char **argv) {
    // Command line ROS arguments
    ros::init(argc, argv, "detect_objects");


    // Loop 10 Hz
    ros::Rate loop_rate(10);

    ObjectDetector objectDetector;

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();

        objectDetector.findObjects();
    }

    return 0;
}
