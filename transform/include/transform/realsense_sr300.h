#include "ros/ros.h"

#include "sensor_msgs/Image.h"

class Sr300 {
private:
	std::string topic;

    int bottomLimit, topLimit, leftLimit, rightLimit;

    ros::Publisher pub;

public:
    Sr300(std::string topic, int bottomLimit, int topLimit, int leftLimit, int rightLimit);
    
    // Copy constructor
    Sr300(const Sr300 & other);
    
    // Another copy
    Sr300(Sr300 & other);

    void callback(const sensor_msgs::Image::ConstPtr & msg);
};
