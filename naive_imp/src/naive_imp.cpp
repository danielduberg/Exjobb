#include "ros/ros.h"

// C++ general
#include "vector"

// Msgs
#include "sensor_msgs/Image.h"

// Own
#include "naive_imp/sensor.h"

// Resolution
// Distance


std::vector<Sensor> sensors;

// TODO
void publishDistances(ros::Publisher & pub) {
    // Do something so that if it has taken a long time the distance gets very close!

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world ";
    msg.data = ss.str();

    pub.publish(msg);
}

void initSubscribers(ros::NodeHandle nh, std::vector<ros::Subscriber> & subs) {
    std::vector<float> offsets;
    std::vector<float> angles;
    std::vector<std::string> topics;

    nh.getParam("/config/offsets", offsets);
    nh.getParam("/config/angles", angles);
    nh.getParam("/config/topics", topics);

    for (size_t i = 0; i < topics.size(); i++) {
        sensors.push_back(Sensor(offsets[i], angles[i], topics[i]));
        subs.push_back(nh.subscribe(topics[i], 10, &Sensor::callback, &sensors[i]));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "naive_imp");

    ros::NodeHandle nh;

    std::vector<ros::Subscriber> subs;

    initSubscribers(nh, subs);


    ros::Publisher pub = nh.advertise<std_msgs::String>("distances", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        publishDistances(pub);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
