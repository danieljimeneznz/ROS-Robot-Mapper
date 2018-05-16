//
// Created by dan on 16/05/18.
//

#ifndef PROJECT_OBJECT_DETECTION_H
#define PROJECT_OBJECT_DETECTION_H

#include <opencv_apps/Line.h>
#include <tf/LinearMath/Vector3.h>

/**
 * This namespace contains useful helper functions.
 * Header file contains implementation as it was easier to copy it here than get catkin to try to build the lib.
 */
namespace object_detection {
    // CONSTANTS
    static unsigned int multiplier = 100;
    static unsigned int xAxisOffset = 6;
    static double centreThreshold = 20.0;


    tf::Vector3 averageVector(const tf::Vector3 a, const tf::Vector3 b) {
        return (a + b) * 0.5;
    }

    /**
     * Determines if two lines are perpendicular, returns the vector point where the intersect if they do.
     *
     * @param line1     First line to check.
     * @param line2     Second line to check.
     * @return          The point where they intersect or nullptr if they don't.
     */
    tf::Vector3 linesArePerpendicular(const opencv_apps::Line line1, const opencv_apps::Line line2) {
        // Convert line points into vector.
        tf::Vector3 a[2], b[2];
        a[0] = tf::Vector3(line1.pt1.x, line1.pt1.y, 0);
        a[1] = tf::Vector3(line1.pt2.x, line1.pt2.y, 0);

        // Convert line points into vector.
        b[0] = tf::Vector3(line2.pt1.x, line2.pt1.y, 0);
        b[1] = tf::Vector3(line2.pt2.x, line2.pt2.y, 0);

        // Get a vector that represents each of the two lines.
        tf::Vector3 aa = a[0] - a[1];
        tf::Vector3 bb = b[0] - b[1];

        // If the dot product of the two vectors is less than threshold/close to zero, they are perpendicular.
        if (fabs(tf::tfDot(aa, bb)) < 5.0) {
            for (int i = 0; i < 2; i++) { // NOLINT
                for (int j = 0; j < 2; j++) { // NOLINT
                    // If distance is less than threshold then the two points are close to each other.
                    if (fabs(tf::tfDistance(a[i], b[j])) < 5.0) {
                        // We have two lines that are perpendicular and close to each other.
                        // Return the average point where the two vectors meet.
                        return object_detection::averageVector(a[i], b[j]);
                    }
                }
            }
        }
        return tf::Vector3();
    }

    /**
     * Get the vectors that represents the sides of the objects. i.e. A & B
     *
     * A        B
     * x        x
     *  \      /
     *   \    /
     *    \  /
     *     x
     *     C
     *
     * @param line1     First line of object.
     * @param line2     Second line of object.
    * @param sides      The two vectors representing the sides of an object. A & B as an array[2].
     */
    void getObjectSides(const opencv_apps::Line line1, const opencv_apps::Line line2, tf::Vector3 sides[2]) {
        // Convert line points into vector.
        tf::Vector3 a[2], b[2];
        a[0] = tf::Vector3(line1.pt1.x, line1.pt1.y, 0);
        a[1] = tf::Vector3(line1.pt2.x, line1.pt2.y, 0);

        // Convert line points into vector.
        b[0] = tf::Vector3(line2.pt1.x, line2.pt1.y, 0);
        b[1] = tf::Vector3(line2.pt2.x, line2.pt2.y, 0);

        for (int i = 0; i < 2; i++) { // NOLINT
            for (int j = 0; j < 2; j++) { // NOLINT
                // If distance is less than threshold then the two points are close to each other.
                if (fabs(tf::tfDistance(a[i], b[j])) < 5.0) {
                    // We have two lines that are close to each other.
                    // Therefore the other two vectors are the ones we wish to return.
                    int sideAIndex = (i == 1) ? 0 : 1;
                    int sideBIndex = (j == 1) ? 0 : 1;
                    sides[0] = a[sideAIndex];
                    sides[1] = b[sideBIndex];
                }
            }
        }
    }
}

#endif //PROJECT_OBJECT_DETECTION_H
