#include "ros/ros.h"

#include "sensor_msgs/Image.h"

#include <image_transport/image_transport.h>

#include "interface_image_view/image.h"
#include "exjobb_msgs/View.h"

#include <cv_bridge/cv_bridge.h>

#include "exjobb_msgs/Control.h"

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

float look_direction_;

float error_ = 0;

void controlCallback(const exjobb_msgs::Control::ConstPtr & msg) {
    look_direction_ = msg->look_direction;
}

/**
 * @brief getDirectionDistance
 * @param direction_1
 * @param direction_2
 * @return return the +distance if direction_1 is to the right of direction_2 else -distance
 */
float getDirectionDistance(float direction_1, float direction_2) {
    float distance = direction_2 - direction_1;

    if (distance < -180) {
        return distance + 360;
    }

    if (distance > 180) {
        return distance - 360;
    }

    return distance;
}

Image * getLeftClosestImage(const std::vector<Image *> & images, float look_direction) {
    int minDistance = 370;
    int index = -1;

    for (size_t i = 0; i < images.size(); i++) {
        float distance = getDirectionDistance(look_direction, images[i]->look_direction_);
        if (distance >= 0 && distance < minDistance) {
            minDistance = distance;
            index = i;
        }
    }

    return images[index];
}

// TODO: Make the above and under to one!

Image * getRightClosestImage(const std::vector<Image *> & images, float look_direction) {
    int minDistance = 370;
    int index = -1;

    for (size_t i = 0; i < images.size(); i++) {
        float distance = getDirectionDistance(images[i]->look_direction_, look_direction);
        if (distance >= 0 && distance < minDistance) {
            minDistance = distance;
            index = i;
        }
    }

    return images[index];
}

void publish(const image_transport::Publisher & pub, const std::vector<Image *> & images, float fov, float look_direction, ros::Publisher & view_pub) {
    for (size_t i = 0; i < images.size(); i++) {
        if (images[i]->image_.data.size() == 0) {
            return;
        }
    }

    Image * leftClosestImage = getLeftClosestImage(images, look_direction);
    Image * rightClosestImage = getRightClosestImage(images, look_direction);

    float leftDistance = getDirectionDistance(leftClosestImage->look_direction_, look_direction);
    float rightDistance = getDirectionDistance(rightClosestImage->look_direction_, look_direction);

    /*
    if (std::fabs(leftDistance) < std::fabs(rightDistance)) {
        pub.publish(leftClosestImage->image_);
    } else {
        pub.publish(rightClosestImage->image_);
    }

    return;
    */

    //std::cout << leftDistance << ", " << rightDistance << std::endl;

    cv::Mat image;

    if (leftDistance <= error_ && leftDistance >= -error_) {
        image = cv_bridge::toCvCopy(leftClosestImage->image_, "bgr8")->image;
    } else if (rightDistance <= error_ && rightDistance >= -error_) {
        image = cv_bridge::toCvCopy(rightClosestImage->image_, "bgr8")->image;
    } else {
        cv::Mat leftImage = cv_bridge::toCvCopy(leftClosestImage->image_, "bgr8")->image;
        cv::Mat rightImage = cv_bridge::toCvCopy(rightClosestImage->image_, "bgr8")->image;


        float leftPixelsPerDegree = leftImage.cols / leftClosestImage->fov_;
        float rightPixelsPerDegree = rightImage.cols / rightClosestImage->fov_;

        int leftNumPixelsRemove = std::fabs(leftDistance) * leftPixelsPerDegree;
        int rightNumPixelsRemove = std::fabs(rightDistance) * rightPixelsPerDegree;

        std::vector<cv::Mat> stitch_images;
        stitch_images.push_back(leftImage);
        stitch_images.push_back(rightImage);

        if (!leftClosestImage->stitch(stitch_images, &image))
        {
            ROS_ERROR_STREAM("Could not stitch the images: " << leftClosestImage->topic_ << " and " << rightClosestImage->topic_);
            return;
        }

        image = image(cv::Rect(leftNumPixelsRemove, 0, image.cols - leftNumPixelsRemove - rightNumPixelsRemove, image.rows));

        cv::resize(image, image, cv::Size(), leftImage.cols / (float) image.cols, leftImage.rows / (float) image.rows, cv::INTER_LANCZOS4);

        ROS_ERROR_STREAM(image.cols << "x" << image.rows);
    }

    sensor_msgs::Image::ConstPtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    pub.publish(msg);

    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "interface_image_view");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    std::vector<std::string> sub_topics;
    nh.getParam("sub_topics", sub_topics);
    std::vector<float> look_direction, image_fov;
    nh.getParam("look_direction", look_direction);
    nh.getParam("image_fov", image_fov);

    std::vector<std::string> image_path;
    nh.getParam("image_path", image_path);

    std::vector<Image *> images;

    image_transport::Subscriber image_subs[sub_topics.size()];

    for (size_t i = 0; i < sub_topics.size(); i++) {
        Image * image = new Image(sub_topics[i], look_direction[i], image_fov[i], image_path[i]);
        images.push_back(image);
        image_subs[i] = it.subscribe(sub_topics[i], 1000, &Image::callback, images.back());
    }

    std::string pub_topic;
    nh.param<std::string>("publish_topic_name", pub_topic, "interface_image_view");
    image_transport::Publisher pub = it.advertise(pub_topic, 1000);

    ros::Publisher view_pub = nh.advertise<exjobb_msgs::View>("view", 1000);

    ros::Subscriber control_sub = nh.subscribe<exjobb_msgs::Control>("/control", 1000, controlCallback);

    float fov;
    nh.param<float>("wanted_fov", fov, 180);

    float frequency;
    nh.param<float>("frequency", frequency, 10);

    ros::Rate rate(frequency);
    while (ros::ok()) {
        ros::spinOnce();

        publish(pub, images, fov, look_direction_, view_pub);

        rate.sleep();
    }

    return 0;
}
