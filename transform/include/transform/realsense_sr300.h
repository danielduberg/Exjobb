#include "ros/ros.h"

#include "sensor_msgs/Image.h"

class Sr300 {
private:
    float offset, angle;

    std::string topic;

    int bottomLimit, topLimit, leftLimit, rightLimit;

    ros::Publisher pub;

public:
    Sr300(ros::NodeHandle nh, float offset, float angle, std::string topic, int bottomLimit, int topLimit, int leftLimit, int rightLimit);

    void callback(const sensor_msgs::Image::ConstPtr & msg);
};
