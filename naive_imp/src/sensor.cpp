#include <ros/ros.h>

#include "naive_imp/sensor.h"

Sensor::Sensor(float offset, float angle, std::string topic)
    : offset(offset)
    , angle(angle)
    , topic(topic) {
    lastUpdate = ros::Time::now();
}

// TODO
void Sensor::callback(const std_msgs::String::ConstPtr &msg) {
    lastUpdate = ros::Time::now();
}

float Sensor::timeSinceLastUpdate() {
    return (ros::Time::now() - lastUpdate).toSec();
}


const std::vector<float> & Sensor::getDistances() {
    return distances;
}
