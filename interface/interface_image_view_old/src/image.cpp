#include <ros/ros.h>
#include <ros/package.h>

#include "interface_image_view/image.h"

#include "sensor_msgs/Image.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

Image::Image(std::string topic, float direction, float fov, std::string image_path)
    : topic_(topic)
    , look_direction_(direction)
    , fov_(fov)
    , stitcher_(cv::Stitcher::createDefault(true)) {

    std::vector<cv::Mat> images;

    std::string path = ros::package::getPath("interface_image_view") + "/" + image_path;

    cv::Mat image = cv::imread(path + "/left.png", CV_LOAD_IMAGE_UNCHANGED);   // Read the file

    // Check for invalid input
    if(!image.data) {
        std::cout <<  "Could not open or find the image" << std::endl;
        return;
    }

    images.push_back(image);

    image = cv::imread(path + "/right.png", CV_LOAD_IMAGE_UNCHANGED);   // Read the file

    // Check for invalid input
    if(!image.data) {
        std::cout <<  "Could not open or find the image" << std::endl;
        return;
    }

    stitcher_ready_ = true;

    images.push_back(image);

    cv::Stitcher::Status status = stitcher_.estimateTransform(images);

    //if (cv::Stitcher::OK != status) {
        std::cout << image_path <<  " status: " << status << std::endl;
    //}
}

void Image::callback(const sensor_msgs::Image::ConstPtr & msg) {
    image_ = *msg;
}
