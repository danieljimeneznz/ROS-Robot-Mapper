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
    Barrel() {
        this->type = "Barrel";
    }

    double radius;
};

class Container : Object {
public:
    Container() {
        this->type = "Container";
    }

    double width;
    double height;
};

class ObjectDetector {
public:
    ObjectDetector() {
        // Subscribe to the line and circle topics.
        lines_sub = handle.subscribe("/object_detection/lines", 1, &ObjectDetector::linesCallback, this);
        circle_sub = handle.subscribe("/hough_circles/circles", 1, &ObjectDetector::circlesCallback, this);

        multiplier = 100.0;
        binThreshold = 0.1;
        threshold = 100;
    }

    void linesCallback(const opencv_apps::LineArrayStampedConstPtr &lineData) {
        // Convert lines into a box centre co-ordinate if the lines are almost perpendicular to each other.
        // if the lines are too long then assume walls. i.e. > 1m or 100px.
        // Get centre co-ordinate referenced to the global frame.
        // add object to bin.

        // container = centre, length and width.
        unsigned int pointThreshold = 5;

        for (int i = 0; i < lineData->lines.size(); i++) {
            opencv_apps::Line line1 = lineData->lines[i];
            for (int j = 0; j < lineData->lines.size(); j++) {
                opencv_apps::Line line2 = lineData->lines[j];

                if (line1.pt1.x == line2.pt1.x && line1.pt1.y == line2.pt1.y && line1.pt2.x == line2.pt2.x &&
                    line1.pt2.y == line2.pt2.y) {
                    // Same line, ignore.
                    continue;
                }

                int pt11xDiff = abs(int(line1.pt1.x - line2.pt1.x));
                int pt11yDiff = abs(int(line1.pt1.y - line2.pt1.y));
                int pt22xDiff = abs(int(line1.pt2.x - line2.pt2.x));
                int pt22yDiff = abs(int(line1.pt2.y - line2.pt2.y));
                int pt12xDiff = abs(int(line1.pt1.x - line2.pt2.x));
                int pt12yDiff = abs(int(line1.pt1.y - line2.pt2.y));
                int pt21xDiff = abs(int(line1.pt2.x - line2.pt1.x));
                int pt21yDiff = abs(int(line1.pt2.y - line2.pt1.y));


                // See if the x and y co-ordinates of two lines are near each other.
                if ((pt11xDiff < pointThreshold && pt11yDiff < pointThreshold) ||
                    (pt22xDiff < pointThreshold && pt22yDiff < pointThreshold) ||
                    (pt12xDiff < pointThreshold && pt12yDiff < pointThreshold) ||
                    (pt21xDiff < pointThreshold && pt12yDiff < pointThreshold)) {

                    // The lines are close to each other. Therefore these two lines represent two sides of a container.
                    Container *container = new Container();
                    container->width =
                            sqrt(pow(line1.pt2.x - line1.pt1.x, 2) + pow(line1.pt2.y - line1.pt2.y, 2)) / multiplier;
                    container->height =
                            sqrt(pow(line2.pt2.x - line2.pt1.x, 2) + pow(line2.pt2.y - line2.pt2.y, 2)) / multiplier;

                    // Set the x and y co-ordinates as x = x_min +
                    // TODO: Figure this out.
                }
            }
        }
    }

    void circlesCallback(const opencv_apps::CircleArrayStampedConstPtr &circleData) {
        // Get the centre co-ordinate referenced to the global frame.
//        for (int i = 0; i < circleData->circles.size())
        // TODO: Figure this out.

        // add object to bin.
        // barrel = centre, radius.
    }

    void referencePointsToGlobalFrame(double &x, double &y) {
    }

    /**
     * Adds an object to the bin or increments the counter if it is already in there (granted that the type of the object
     * is the same as the one currently suggested at that point).
     *
     * @param object    The object to insert into the object bin.
     */
    void addObjectToBin(const Object &object) {
        bool bInBin = false;
        unsigned int counterIndex = 0;
        // Place in the bin if not already in there.
        for (unsigned int i = 0; i < objectBin.size(); i++) {
            Object obj = objectBin[i];

            if (fabs(object.x - obj.x) < binThreshold && fabs(object.y - obj.y) < binThreshold &&
                obj.type != object.type) {
                bInBin = true;
                counterIndex = i;
                break;
            }
        }

        // Add object to bin if it was not found.
        if (!bInBin) {
            objectBin.push_back(object);
            objectBinCount.push_back(1);
        } else {
            // Increment bin counter.
            objectBinCount[counterIndex]++;
        }
    }

    void findObjects() {
        // Consolidate objects from the bin.
        // Add the found object to the object map if it does not already exist.
        // Display found object to console.
    }

private:
    ros::NodeHandle handle;

    ros::Subscriber lines_sub;
    ros::Subscriber circle_sub;

    double multiplier;

    // x and y coordinates relative to the global reference frame.
    // Objects stored as map i.e. { Id: Object }
    std::map<int, Object> objects;

    // Object Bin represents all the possible objects that could exist in the world at that point (box or circle).
    std::vector<Object> objectBin;
    double binThreshold;
    // Object Bin count is a counter that counts how many possible times the object has been detected as a box or circle.
    std::vector<unsigned int> objectBinCount;
    unsigned int threshold;

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
