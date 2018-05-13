//
// Created by dan on 13/05/18.
//

#include <ros/ros.h>
#include <opencv_apps/LineArrayStamped.h>

class LineCleaner {
public:

    /**
 * Create a new ScanCleaner instance.
 */
    LineCleaner() {
        // Publish the clean scan information to the /clean/lines topic
        lines_pub = handle.advertise<opencv_apps::LineArrayStamped>("/clean/lines", 1);

        // Subscribe to the lines topic.
        lines_sub = handle.subscribe("/dirty/lines", 1, &LineCleaner::houghLinesCallback, this);
    }

    /**
     * Cleans up the lines by grouping similar lines/parallel lines into a single line segment.
     *
     * @param lineData      The line data to tidy up.
     */
    void houghLinesCallback(const opencv_apps::LineArrayStampedConstPtr &lineData) {
        // Lines that are next to each other need to be collated to find at least two sides of the object.
        std::vector <opencv_apps::Line> lines;
        opencv_apps::LineArrayStamped linesMsg;

        // TODO: REWRITE METHOD.

        int d1x12x1Diff;
        int d1y12y1Diff;
        int d1x22x2Diff;
        int d1y22y2Diff;
        int thres = 10;

        for (int i = 0; i < lineData->lines.size(); i++) {
            bool bDifferent = true;
            opencv_apps::Line line = lineData->lines[i];
            for (int j = 0; j < lines.size(); j++) {
                // The delta difference is currentLine_x/lines_x + currentLine_y/lines_y
                // The closer this value is to 2, the closer these two points are to being equal to each other.
                d1x12x1Diff = abs(int(line.pt1.x - lines[j].pt1.x));
                d1y12y1Diff = abs(int(line.pt1.y - lines[j].pt1.y));
                d1x22x2Diff = abs(int(line.pt2.x - lines[j].pt2.x));
                d1y22y2Diff = abs(int(line.pt2.y - lines[j].pt2.y));

                ROS_DEBUG("%d %d %d %d", d1x12x1Diff, d1y12y1Diff, d1x22x2Diff, d1y22y2Diff);

                if (d1x12x1Diff < thres && d1y12y1Diff < thres && d1x22x2Diff < thres && d1y22y2Diff < thres) {
                    // We have a new line that is the same as the current one.
                    bDifferent = false;
                }
            }
            if (bDifferent) {
                lines.push_back(line);
                linesMsg.lines.push_back(line);
                ROS_DEBUG("Line found x1,y1: %.4f,%.4f x2,y2: %.4f,%.4f", line.pt1.x, line.pt1.y, line.pt2.x,
                          line.pt2.y);
            }
        }

        lines_pub.publish(linesMsg);
    }

private:
    ros::NodeHandle handle;

    ros::Publisher lines_pub;
    ros::Subscriber lines_sub;
};

int main(int argc, char **argv) {
    // Command line ROS arguments
    ros::init(argc, argv, "line_cleaner");

    LineCleaner lineCleaner;
    ros::spin();

    return 0;
}