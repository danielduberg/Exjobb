#include "ros/ros.h"

// OpenCV
#include <cv_bridge/cv_bridge.h>

// C++
#include <string>

// Own
#include "transform/realsense_sr300.h"
#include "exjobb_msgs/Distance.h"

Sr300::Sr300(std::string topic, int bottomLimit, int topLimit, int leftLimit, int rightLimit)
    : topic(topic)
    , bottomLimit(bottomLimit)
    , topLimit(topLimit)
    , leftLimit(leftLimit)
    , rightLimit(rightLimit) {

    ros::NodeHandle nh;

    // TODO: Update
    pub = nh.advertise<exjobb_msgs::Distance>(topic, 1000);

    std::cout << "Epic: " << topic << ", " << bottomLimit << ", " << topLimit << ", " << leftLimit << ", " << rightLimit << std::endl;
}

Sr300::Sr300(const Sr300 & other)
    : topic(other.topic)
    , bottomLimit(other.bottomLimit)
    , topLimit(other.topLimit)
    , leftLimit(other.leftLimit)
    , rightLimit(other.rightLimit) {


    ros::NodeHandle nh;

    pub = nh.advertise<exjobb_msgs::Distance>(topic, 1000);

    std::cout << "Copy 1: " << topic << ", " << bottomLimit << ", " << topLimit << ", " << leftLimit << ", " << rightLimit << std::endl;
}

Sr300::Sr300(Sr300 & other)
    : topic(other.topic)
    , bottomLimit(other.bottomLimit)
    , topLimit(other.topLimit)
    , leftLimit(other.leftLimit)
    , rightLimit(other.rightLimit) {


    ros::NodeHandle nh;

    pub = nh.advertise<exjobb_msgs::Distance>(topic, 1000);

    std::cout << "Copy 2: " << topic << ", " << bottomLimit << ", " << topLimit << ", " << leftLimit << ", " << rightLimit << std::endl;
}

void Sr300::callback(const sensor_msgs::Image::ConstPtr & msg) {
    // std::cout << topic << ", " << bottomLimit << ", " << topLimit << ", " << leftLimit << ", " << rightLimit << std::endl;

    cv_bridge::CvImagePtr test = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

    cv::Mat image = test->image;

    std::vector<float> distances;

    // TODO: Check so everything is alright. at can overflow or something?!
    for (size_t y = topLimit; y < bottomLimit; y++) {
        for (size_t x = leftLimit; x < rightLimit; x++) {
            float distance = image.at<unsigned short int>(y, x);
            if (distance == 0) {
                distance = std::numeric_limits<float>::infinity();
            }
            if (y == topLimit) {
                distances.push_back(distance);
            } else {
                if (distance < distances[x - leftLimit]) {
                    distances[x - leftLimit] = distance;
                }
            }
        }
    }

    exjobb_msgs::Distance distance_msg;

    // TODO: Add header
    distance_msg.header = msg->header;
    distance_msg.distances = distances;

    pub.publish(distance_msg);


    /*
    for (size_t i = (distances.size() / 2) - 10; i < (distances.size() / 2) + 10; i++) {
        std::cout << distances[i] << " ";
    }
    std::cout << std::endl;
    */
    //std::cout << (float) test->image.at<unsigned short int>(test->image.rows / 2, test->image.cols / 2) << std::endl;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "transform_realsense_sr300");

    ros::NodeHandle nh;

    std::vector<std::string> topics;
    nh.getParam("/topics", topics);

    std::vector<std::string> param_names;
    nh.getParamNames(param_names);

    for (size_t i = 0; i < param_names.size(); i++) {
        std::cout << param_names[i] << std::endl;
    }

    // List because vector moves the elements when new one is inserted
    std::list<Sr300> sr300;
    std::list<ros::Subscriber> subs;

    for (size_t i = 0; i < topics.size(); i++) {
        // TODO
        sr300.push_back(Sr300(topics[i], 240, 0, 0, 640));
        std::cout << topics[i] << std::endl;
        std::cout << std::to_string(i+1) << std::endl;
        subs.push_back(nh.subscribe("camera" + std::to_string(i+1) + "/depth/image_raw", 100, &Sr300::callback, &sr300.back()));
    }

    ros::spin();

    return 0;
}
