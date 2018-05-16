//
// Created by dan on 16/05/18.
//

#ifndef PROJECT_OBJECT_DETECTION_H
#define PROJECT_OBJECT_DETECTION_H

#include <opencv_apps/Line.h>
#include <tf/LinearMath/Vector3.h>

/**
 * This namespace contains useful helper functions.
 */
namespace ObjectDetection {
    // CONSTANTS
    static unsigned int multiplier = 100;
    static unsigned int xAxisOffset = 6;
    static double centreThreshold = 5.0;


    /**
     * Determines if two lines are perpendicular, returns the vector point where the intersect if they do.
     *
     * @param line1     First line to check.
     * @param line2     Second line to check.
     * @return          The point where they intersect or nullptr if they don't.
     */
    tf::Vector3 linesArePerpendicular(const opencv_apps::Line line1, const opencv_apps::Line line2);

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
    void getObjectSides(const opencv_apps::Line line1, const opencv_apps::Line line2, tf::Vector3 sides[2]);

    tf::Vector3 averageVector(const tf::Vector3 a, const tf::Vector3 b);
}

#endif //PROJECT_OBJECT_DETECTION_H
