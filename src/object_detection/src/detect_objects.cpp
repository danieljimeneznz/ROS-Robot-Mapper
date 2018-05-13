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

        multiplier = 100.0;
        binThreshold = 0.1;
        threshold = 1;
    }

    void borderCallback(const opencv_apps::LineArrayStampedConstPtr &border) {

    }

    void linesCallback(const opencv_apps::LineArrayStampedConstPtr &lineData) {
        // Convert lines into a box centre co-ordinate if the lines are almost perpendicular to each other.
        // if the lines are too long then assume walls. i.e. > 1m or 100px.
        // Get centre co-ordinate referenced to the global frame.
        // add object to bin.

        // container = centre, length and width.
        unsigned int pointThreshold = 10;

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

                Container *container = new Container();
                bool bLinesClose = false;
                double a = 0.0, b = 0.0, c = 0.0, d = 0.0, e = 0.0, f = 0.0;
                // See if the x and y co-ordinates of two lines are near each other.
                // Set the x and y co-ordinates as x = a + (a - e)/2 + (c - a)/2, y  = b + (b - f)/2 + (d - b)/2
                // Where a,b = point of inflection, c,d = other end of a line and e,f = other end of the other line.
                if (pt11xDiff < pointThreshold && pt11yDiff < pointThreshold) {
                    // Intercept at line1.pt1 && line2.pt1
                    a = line1.pt1.x;
                    b = line1.pt1.y;
                    c = line1.pt2.x;
                    d = line1.pt2.y;
                    e = line2.pt2.x;
                    f = line2.pt2.y;
                    bLinesClose = true;
                }
//                } else if (pt22xDiff < pointThreshold && pt22yDiff < pointThreshold) {
//                    // Intercept at line1.pt2 && line2.pt2
//                    a = line1.pt2.x;
//                    b = line1.pt2.y;
//                    c = line1.pt1.x;
//                    d = line1.pt1.y;
//                    e = line2.pt1.x;
//                    f = line2.pt1.y;
//                    bLinesClose = true;
//                } else if (pt12xDiff < pointThreshold && pt12yDiff < pointThreshold) {
//                    // Intercept at line1.pt1 && line2.pt2
//                    a = line1.pt1.x;
//                    b = line1.pt1.y;
//                    c = line1.pt2.x;
//                    d = line1.pt2.y;
//                    e = line2.pt1.x;
//                    f = line2.pt1.y;
//                    bLinesClose = true;
//                } else if (pt21xDiff < pointThreshold && pt12yDiff < pointThreshold) {
//                    // Intercept at line1.pt2 && line2.pt1
//                    a = line1.pt2.x;
//                    b = line1.pt2.y;
//                    c = line1.pt1.x;
//                    d = line1.pt1.y;
//                    e = line2.pt2.x;
//                    f = line2.pt2.y;
//                    bLinesClose = true;
//                }

                if (bLinesClose) {
                    // The lines are close to each other. Therefore these two lines represent two sides of a container.
                    container->x = (a + (a - e) / 2 + (c - a) / 2) / multiplier - 6;
//                    container->x = (a + e + c) / multiplier - 6;

                    container->y = (b + (b - f) / 2 + (d - b) / 2) / multiplier;
//                    container->y = (b + f - b) / multiplier;


                    container->width =
                            sqrt(pow(line1.pt2.x - line1.pt1.x, 2) + pow(line1.pt2.y - line1.pt2.y, 2)) / multiplier;
                    container->height =
                            sqrt(pow(line2.pt2.x - line2.pt1.x, 2) + pow(line2.pt2.y - line2.pt2.y, 2)) / multiplier;

                    referencePointsToGlobalFrame(&container->x, &container->y);
                    addObjectToBin(container);
                }
            }
        }
    }

    void circlesCallback(const opencv_apps::CircleArrayStampedConstPtr &circleData) {
        for (int i = 0; i < circleData->circles.size(); i++) {
            Barrel *barrel = new Barrel();
            barrel->x = circleData->circles[i].center.x / multiplier - 6;
            barrel->y = circleData->circles[i].center.y / multiplier;
            barrel->radius = circleData->circles[i].radius / multiplier;

            // Get the centre co-ordinate referenced to the global frame.
            referencePointsToGlobalFrame(&barrel->x, &barrel->y);
            addObjectToBin(barrel);
        }
    }

    void referencePointsToGlobalFrame(double *x, double *y) {
    }

    /**
     * Adds an object to the bin or increments the counter if it is already in there (granted that the type of the object
     * is the same as the one currently suggested at that point).
     *
     * @param object    The object to insert into the object bin.
     */
    void addObjectToBin(Object *object) {
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

    double multiplier;

    // x and y coordinates relative to the global reference frame.
    // Objects stored as vector i.e. index = Id, value = *Object}
    std::vector<Object *> objects;

    // Object Bin represents all the possible objects that could exist in the world at that point (box or circle).
    std::vector<Object *> objectBin;
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
