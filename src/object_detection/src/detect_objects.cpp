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
    tf::Vector3 centre;

    explicit Object(tf::Vector3 centre) {
        this->type = "";
        // Need to reduce the centre to be the correct Vector (as current vectors are multiplied).
        // Switch x and y as the robot moves in the x direction but we treat this as y in the image.
        this->centre = tf::Vector3(centre.y() / object_detection::multiplier,
                centre.x() / object_detection::multiplier - object_detection::xAxisOffset, 0);
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
    explicit Barrel(tf::Vector3 centre) : Object(centre) {
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
    explicit Container(tf::Vector3 centre) : Object(centre) {
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

        objectsCurrentSize = 0;
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
                closestObjectPoint = object_detection::linesArePerpendicular(lineData->lines[i], lineData->lines[j]);

                if (closestObjectPoint) {
                    tf::Vector3 sides[2];
                    object_detection::getObjectSides(lineData->lines[i], lineData->lines[j], sides);
                    Container *container = new Container(object_detection::averageVector(sides[0], sides[1]));

                    // Sides should be contained in an array inside the obj.
                    container->width = tf::tfDistance(container->centre, sides[0]);
                    container->height = tf::tfDistance(container->centre, sides[1]);

                    referencePointsToGlobalFrame(container);
                    addObject(container);
                }
            }
        }
    }

    void circlesCallback(const opencv_apps::CircleArrayStampedConstPtr &circleData) {
        for (int i = 0; i < circleData->circles.size(); i++) {
            Barrel *barrel = new Barrel(tf::Vector3(circleData->circles[i].center.x, circleData->circles[i].center.y, 0));
            // This should take the average radius!
            barrel->radius = circleData->circles[i].radius / object_detection::multiplier;

            // Get the centre co-ordinate referenced to the global frame.
            referencePointsToGlobalFrame(barrel);
            addObject(barrel);
        }
    }

    void referencePointsToGlobalFrame(Object* object) {
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
        // Ignore Walls and large/small containers/barrels.
        if (object->type == "Container") {
            if (object->getWidth() > 1.1 || object->getHeight() > 1.1 ||
                object->getWidth() < 0.08 || object->getHeight() < 0.08) {
                return;
            }
        } else if (object->type == "Barrel") {
            if (object->getRadius() > 0.6 || object->getRadius() < 0.08) {
                return;
            }
        }

        bool bObjectExists = false;
        long replaceObjectIndex = -1;
        for (unsigned int i = 0; i < objects.size(); i++) { // NOLINT
            // Place in the bin if not already in there.
            if (object->centre == objects[i]->centre) {
                continue;
            }

            // Otherwise adjust measurements to become more accurate.
            if (tf::tfDistance(object->centre, objects[i]->centre) < object_detection::centreThreshold) {
                // Containers overwrite Barrels.
                if (objects[i]->type == "Barrel" && object->type == "Container") {
                    replaceObjectIndex = i;
                } else if (objects[i]->type != "Container" && object->type != "Barrel") {
                    objects[i]->centre = object_detection::averageVector(object->centre, objects[i]->centre);
                    bObjectExists = true;
                }
            }
        }

        if (replaceObjectIndex > -1) {
            objects.at(unsigned(replaceObjectIndex)) = object;
        }

        if (!bObjectExists) {
            objects.push_back(object);
        }
    }

    void findObjects() {
        ROS_DEBUG("Looking for Objects.");
        for (unsigned long i = objectsCurrentSize; i < objects.size(); i++) {
            // Display found object to console.
            ROS_INFO("Object Found!! - ID: %d", (objects.size() - 1));
            if (objects[i]->type == "Barrel") {
                ROS_INFO("Type: %s, Radius %.2fm, X: %.2f, Y: %.2f", objects[i]->type.c_str(),
                         objects[i]->getRadius(), objects[i]->centre.x(), objects[i]->centre.y());
            } else if (objects[i]->type == "Container") {
                ROS_INFO("Type: %s, Width: %.2fm, Height %.2fm, X: %.2f, Y: %.2f", objects[i]->type.c_str(),
                         objects[i]->getWidth(), objects[i]->getHeight(), objects[i]->centre.x(), objects[i]->centre.y());
            }
        }
        objectsCurrentSize = objects.size();
    }

private:
    ros::NodeHandle handle;

    ros::Subscriber border_sub;
    ros::Subscriber lines_sub;
    ros::Subscriber circle_sub;

    // Object represents all the possible objects that could exist in the world at that point (box or circle).
    std::vector<Object *> objects;

    unsigned long objectsCurrentSize;
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
