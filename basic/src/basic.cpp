#include "ros/ros.h"

#include "sensor_msgs/Image.h"

// Own
#include "exjobb_msgs/Distance.h"


void imgCallback(const sensor_msgs::Image::ConstPtr & msg) {
    // TODO
}

void distCallback(const exjobb_msgs::Distance::ConstPtr & msg) {
    // TODO
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "naive_imp");

    ros::NodeHandle nh;

    ros::Subscriber dist_sub = nh.subscribe("processed/distances", 10, distCallback);

    ros::Subscriber img_sub = nh.subscribe("...", 10, imgCallback);

    ros::spin();

    return 0;
}
