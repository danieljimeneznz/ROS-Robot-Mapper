//
// Created by dan on 7/05/18.
//
#include <ros/ros.h>
#include <opencv_apps/LineArrayStamped.h>
#include <opencv_apps/CircleArrayStamped.h>
#include <geometry_msgs/Vector3.h>
#include <object_detection/object_detection.h>

class Object {
public:
    std::string type;
    double x;
    double y;

    Object() {
        this->type = "";
        this->x = 0.0;
        this->y = 0.0;
    }

    virtual double getRadius() {
        return 0.0;
    };

    virtual double getWidth() {
        return 0.0;
    }

    virtual double getHeight() {
        return 0.0;
    }
};

class Barrel : public Object {
public:
    Barrel() {
        this->radius = 0.0;
        this->type = "Barrel";
    }

    double radius;

    double getRadius() override {
        return this->radius;
    }
};

class Container : public Object {
public:
    Container() {
        this->width = 0.0;
        this->height = 0.0;
        this->type = "Container";
    }

    double width;
    double height;

    double getWidth() override {
        return this->width;
    }

    double getHeight() override {
        return this->height;
    }
};

class ObjectDetector {
public:
    ObjectDetector() {
        // Subscribe to the line and circle topics.
        border_sub = handle.subscribe("/border", 1, &ObjectDetector::borderCallback, this);
        lines_sub = handle.subscribe("/lines", 1, &ObjectDetector::linesCallback, this);
        circle_sub = handle.subscribe("/circles", 1, &ObjectDetector::circlesCallback, this);
    }

    void borderCallback(const geometry_msgs::Vector3ConstPtr &border) {

    }

    void linesCallback(const opencv_apps::LineArrayStampedConstPtr &lineData) {
        // Convert lines into a box centre co-ordinate if the lines are almost perpendicular to each other.
        // Get centre co-ordinate referenced to the global frame.
        // add object to bin.

        // container = centre, length and width.
        tf::Vector3 closestObjectPoint;
        for (int i = 0; i < lineData->lines.size(); i++) {
            for (int j = 0; j < lineData->lines.size(); j++) {
                closestObjectPoint = ObjectDetection::linesArePerpendicular(lineData->lines[i], lineData->lines[j]);

                if (closestObjectPoint) {
                    Container *container = new Container();
                    tf::Vector3 sides[2];
                    ObjectDetection::getObjectSides(lineData->lines[i], lineData->lines[j], sides);
                    tf::Vector3 center;
                    center = ObjectDetection::averageVector(sides[0], sides[1]);

                    container->x = center.x() / ObjectDetection::multiplier - ObjectDetection::xAxisOffset;
                    container->y = center.y() / ObjectDetection::multiplier;

                    container->width = tf::tfDistance(center, sides[0]) / ObjectDetection::multiplier;
                    container->height = tf::tfDistance(center, sides[1]) / ObjectDetection::multiplier;

                    referencePointsToGlobalFrame(&container->x, &container->y);
                    addObject(container);
                }
            }
        }
    }

    void circlesCallback(const opencv_apps::CircleArrayStampedConstPtr &circleData) {
        for (int i = 0; i < circleData->circles.size(); i++) {
            Barrel *barrel = new Barrel();
            barrel->x = circleData->circles[i].center.x / ObjectDetection::multiplier - ObjectDetection::xAxisOffset;
            barrel->y = circleData->circles[i].center.y / ObjectDetection::multiplier;
            barrel->radius = circleData->circles[i].radius / ObjectDetection::multiplier;

            // Get the centre co-ordinate referenced to the global frame.
            referencePointsToGlobalFrame(&barrel->x, &barrel->y);
            addObject(barrel);
        }
    }

    void referencePointsToGlobalFrame(double *x, double *y) {
        // Switch x and y as the robot moves in the x direction but we treat this as y in the image.
        double temp = *x;
        *x = *y;
        *y = temp;

        // TODO: BELOW.
        // Transform from LaserLink to odom.
    }

    /**
     * Adds an object to the bin or increments the counter if it is already in there (granted that the type of the object
     * is the same as the one currently suggested at that point).
     *
     * @param object    The object to insert into the object bin.
     */
    void addObject(Object *object) {
        bool bInBin = false;
        unsigned int counterIndex = 0;
        // Place in the bin if not already in there.
        for (unsigned int i = 0; i < objectBin.size(); i++) {
            Object *obj = objectBin[i];

            if (fabs(object->x - obj->x) < binThreshold && fabs(object->y - obj->y) < binThreshold &&
                obj->type == object->type) {
                bInBin = true;
                counterIndex = i;
                break;
            }
        }

        // Add object to bin if it was not found.
        if (!bInBin) {
            if (object->type == "Barrel") {
                ROS_DEBUG("Added object: %s to bin. (x, y, r): (%.4f, %.4f, %.4f)", object->type.c_str(), object->x,
                          object->y, object->getRadius());
            } else if (object->type == "Container") {
                ROS_DEBUG("Added object: %s to bin. (x, y, w, h): (%.4f, %.4f, %.4f, %.4f)", object->type.c_str(),
                          object->x, object->y, object->getWidth(), object->getHeight());
            }
            objectBin.push_back(object);
            objectBinCount.push_back(1);
        } else {
            // Don't want to fill the bin up too much if we already know the object.
            if (objectBinCount[counterIndex] < threshold * 2) {
                // Increment bin counter.
                objectBinCount[counterIndex]++;
            }
            // delete the object to free memory.
            delete object;
        }
    }

    void findObjects() {
        ROS_DEBUG("Looking for Objects.");
        bool bNewObjectFound = true;
        // Consolidate objects from the bin.
        for (unsigned int i = 0; i < objectBinCount.size(); i++) {
            for (unsigned int j = 0; j < objects.size(); j++) {
                // We have found a valid object, add it to the object map if its not already in there.
                if (objects[j]->x == objectBin[i]->x && objects[j]->y == objectBin[i]->y) {
                    bNewObjectFound = false;
                }
            }

            if (objectBinCount[i] >= threshold && bNewObjectFound) {
                // Ignore Walls and large/small containers/barrels.
                if (objectBin[i]->type == "Container") {
                    if (objectBin[i]->getWidth() > 1.1 || objectBin[i]->getHeight() > 1.1 ||
                        objectBin[i]->getWidth() < 0.08 || objectBin[i]->getHeight() < 0.08) {
                        continue;
                    }
                } else if (objectBin[i]->type == "Barrel") {
                    if (objectBin[i]->getRadius() > 0.6 || objectBin[i]->getRadius() < 0.08) {
                        continue;
                    }
                }

                objects.push_back(objectBin[i]);

                // Display found object to console.
                ROS_INFO("Object Found!! - ID: %d", (objects.size() - 1));
                if (objectBin[i]->type == "Barrel") {
                    ROS_INFO("Type: %s, Radius %.2fm, X: %.2f, Y: %.2f", objectBin[i]->type.c_str(),
                             objectBin[i]->getRadius(), objectBin[i]->x, objectBin[i]->y);
                } else if (objectBin[i]->type == "Container") {
                    ROS_INFO("Type: %s, Width: %.2fm, Height %.2fm, X: %.2f, Y: %.2f", objectBin[i]->type.c_str(),
                             objectBin[i]->getWidth(), objectBin[i]->getHeight(), objectBin[i]->x, objectBin[i]->y);
                }
            }
        }
    }

private:
    ros::NodeHandle handle;

    ros::Subscriber border_sub;
    ros::Subscriber lines_sub;
    ros::Subscriber circle_sub;

    // x and y coordinates relative to the global reference frame.
    // Objects stored as vector i.e. index = Id, value = *Object}
    std::vector<Object *> foundObjects;

    // Object Bin represents all the possible objects that could exist in the world at that point (box or circle).
    std::vector<Object *> objects;
};

int main(int argc, char **argv) {
    // Command line ROS arguments
    ros::init(argc, argv, "detect_objects");

    ObjectDetector objectDetector;

    // Loop 10 Hz
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();

        objectDetector.findObjects();
    }

    return 0;
}
