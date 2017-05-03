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
    , stitcher_(cv::Stitcher::createDefault(false))
    , stitcher_ready_(false)
    {

    configureStitcher();

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

    images.push_back(image);

    //stitch(images);
}

void Image::configureStitcher()
{
    //stitcher_.setRegistrationResol(1.0);
    //stitcher_.setCompositingResol(0.1);
    //stitcher_.setFeaturesFinder(new cv::detail::OrbFeaturesFinder(cv::Size(3,1),300));


    stitcher_.setWarper(new cv::CylindricalWarper());
    stitcher_.setWaveCorrection(false);
    stitcher_.setSeamEstimationResol(0.001);
    stitcher_.setPanoConfidenceThresh(0.1);

    stitcher_.setSeamFinder(new cv::detail::GraphCutSeamFinder(cv::detail::GraphCutSeamFinderBase::COST_COLOR_GRAD));
    //stitcher_.setSeamFinder(new cv::detail::NoSeamFinder());
    //stitcher_.setBlender(cv::detail::Blender::createDefault(cv::detail::Blender::NO, true));
    //stitcher_.setExposureCompensator(cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::NO));
    stitcher_.setExposureCompensator(new cv::detail::NoExposureCompensator());
}

bool Image::stitch(std::vector<cv::Mat> & images, cv::Mat * pano_image)
{
    if (stitcher_ready_ && pano_image != NULL)
    {
        stitcher_.composePanorama(images, *pano_image);
    }
    else if (!stitcher_ready_ && pano_image == NULL)
    {
        cv::Stitcher::Status status = stitcher_.estimateTransform(images);

        if (status == cv::Stitcher::OK) {
            stitcher_ready_ = true;
        }
        else
        {
            ROS_ERROR_STREAM("Could not stitch images, error code: " << status);
        }
    }
    else if (!stitcher_ready_ && pano_image != NULL)
    {
        cv::Stitcher::Status status = stitcher_.stitch(images, *pano_image);


        if (status == cv::Stitcher::OK) {
            stitcher_ready_ = true;
        }
        else
        {
            ROS_ERROR_STREAM("Could not stitch images, error code: " << status);
        }
    }

    return stitcher_ready_;
}

void Image::callback(const sensor_msgs::Image::ConstPtr & msg) {
    image_ = *msg;
}
