//
// Created by dan on 16/05/18.
//

#include <object_detection/object_detection.h>

static tf::Vector3 ObjectDetection::linesArePerpendicular(const opencv_apps::Line line1, const opencv_apps::Line line2) {
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
                    return ObjectDetection::averageVector(a[i], b[j]);
                }
            }
        }
    }
    return nullptr;
}

void ObjectDetection::getObjectSides(const opencv_apps::Line line1, const opencv_apps::Line line2, tf::Vector3 sides[2]) {
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


static tf::Vector3 ObjectDetection::averageVector(const tf::Vector3 a, const tf::Vector3 b) {
    return (a + b) * 0.5;
}
