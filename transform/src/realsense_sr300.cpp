#include "ros/ros.h"

// OpenCV
#include <cv_bridge/cv_bridge.h>

// Own
#include "transform/realsense_sr300.h"
#include "exjobb_msgs/sensor_distance.h"

Sr300::Sr300(ros::NodeHandle nh, float offset, float angle, std::string topic, int bottomLimit, int topLimit, int leftLimit, int rightLimit)
    : offset(offset)
    , angle(angle)
    , topic(topic)
    , bottomLimit(bottomLimit)
    , topLimit(topLimit)
    , leftLimit(leftLimit)
    , rightLimit(rightLimit) {

    // TODO: Update
    pub = nh.advertise<exjobb_msgs::sensor_distance>(topic, 1000);
}

void Sr300::callback(const sensor_msgs::Image::ConstPtr & msg) {
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

    exjobb_msgs::sensor_distance distance_msg;

    // TODO: Add header
    distance_msg.header = msg->header;
    distance_msg.offset = offset;
    distance_msg.angle = angle;
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

    // TODO
    Sr300 test(nh, 0, 60, "testis", 240, 0, 0, 100);
    ros::Subscriber sub = nh.subscribe("camera3/depth/image_raw", 100, &Sr300::callback, &test);

    ros::spin();

    return 0;
}
