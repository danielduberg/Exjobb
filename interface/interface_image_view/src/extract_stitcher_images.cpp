#include "ros/ros.h"
#include <ros/package.h>

#include "sensor_msgs/Image.h"

#include <image_transport/image_transport.h>

#include "interface_image_view/image.h"
#include "exjobb_msgs/View.h"

#include <cv_bridge/cv_bridge.h>

#include "exjobb_msgs/Control.h"

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include <opencv2/stitching/stitcher.hpp>

cv::Stitcher stitcher = cv::Stitcher::createDefault(true);

std::vector<bool> haveStitched(4, false);

void extract(const std::vector<Image *> & images, std::vector<std::string> & image_path) {
    for (size_t i = 0; i < images.size(); i++) {
        if (images[i]->stitcher_ready_) {
            haveStitched[i] = true;
        }

        if (images[i]->image_.data.size() == 0) {
            return;
        }
    }

    std::vector<cv::Mat> allImages;
    try {
        allImages.push_back(cv_bridge::toCvCopy(images[0]->image_, "bgr8")->image);
        allImages.push_back(cv_bridge::toCvCopy(images[1]->image_, "bgr8")->image);
        allImages.push_back(cv_bridge::toCvCopy(images[2]->image_, "bgr8")->image);
        allImages.push_back(cv_bridge::toCvCopy(images[3]->image_, "bgr8")->image);
    } catch (cv_bridge::Exception & e) {
        return;
    }

    for (size_t i = 0; i < allImages.size(); i++) {
        if (!haveStitched[i]) {
            std::vector<cv::Mat> image;
            image.push_back(allImages[i]);
            if (i == 0) {
                image.push_back(allImages[allImages.size() - 1]);
            } else {
                image.push_back(allImages[i - 1]);
            }

            cv::Stitcher::Status status = stitcher.estimateTransform(image);

            if (cv::Stitcher::OK == status) {
                std::cout << image_path[i] << " succeeded: " << status << std::endl;

                haveStitched[i] = true;

                std::string path = ros::package::getPath("interface_image_view") + "/" + image_path[i];

                cv::imwrite(path + "/left.png", image[0]);
                cv::imwrite(path + "/right.png", image[1]);
            } else {
                std::cout << image_path[i] << " failed: " << status << std::endl;
            }
        }
    }

    for (size_t i = 0; i < haveStitched.size(); i++) {
        if (!haveStitched[i]) {
            return;
        }
    }

    std::exit(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "extract_stitcher_images");

    ros::NodeHandle nh;

    cv::initModule_nonfree();

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

    float frequency;
    nh.param<float>("frequency", frequency, 100);

    ros::Rate rate(frequency);
    while (ros::ok()) {
        ros::spinOnce();

        extract(images, image_path);

        rate.sleep();
    }

    return 0;
}
