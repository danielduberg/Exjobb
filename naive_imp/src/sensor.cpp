#include <ros/ros.h>


// Own
#include "naive_imp/sensor.h"

Sensor::Sensor(float offset, float angle, std::string topic)
    : offset(offset)
    , angle(angle)
    , topic(topic) {
}

Sensor::Sensor(const Sensor & other)
    : offset(other.offset)
    , angle(other.angle)
    , topic(other.topic)
    , distances()
    , lastUpdate() {
    std::cout << "Copy 1: Sensor" << std::endl;
}

Sensor::Sensor(Sensor & other)
    : offset(other.offset)
    , angle(other.angle)
    , topic(other.topic)
    , distances()
    , lastUpdate() {
    std::cout << "Copy 2: Sensor" << std::endl;
}

// TODO
void Sensor::callback(const exjobb_msgs::Distance::ConstPtr & msg) {
    std::cout << "Tjo" << std::endl;

    lastUpdate = ros::Time::now();

    distances = msg->distances;
    std::cout << "Nope" << std::endl;
}

float Sensor::timeSinceLastUpdate() {
    // Will lastUpdate really be zero the first time?
    if (lastUpdate.toSec() == 0) {
        std::cout << "Tjo" << std::endl;
        return -1;
    }
    return (ros::Time::now() - lastUpdate).toSec();
}


const std::vector<float> & Sensor::getDistances() {
    return distances;
}
