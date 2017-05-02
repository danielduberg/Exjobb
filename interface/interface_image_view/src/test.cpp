#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching/stitcher.hpp>

/*
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include <opencv2/stitching/stitcher.hpp>
*/

void imageCallback(const sensor_msgs::Image::ConstPtr & msg, cv::Mat * image)
{
    *image = cv_bridge::toCvShare(msg, "bgr8")->image;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    cv::Mat left_image, right_image;

    //image_transport::Subscriber left_sub = it.subscribe("left/image_raw", 1, boost::bind(imageCallback, _1, &left_image));
    //image_transport::Subscriber right_sub = it.subscribe("right/image_raw", 1, boost::bind(imageCallback, _1, &right_image));

    ros::Rate rate(1000);
    for (size_t i = 0; i < 10 && ros::ok(); i++)
    {
        rate.sleep();
        ros::spinOnce();
    }

    std::string path = ros::package::getPath("interface_image_view") + "/";

    left_image = cv::imread(path + "left1.png");
    right_image = cv::imread(path + "right1.png");

    std::vector<cv::Mat> images;
    images.push_back(left_image);
    images.push_back(right_image);


    image_transport::Publisher pub = it.advertise("pano", 1);

    cv::Stitcher stitcher = cv::Stitcher::createDefault(false);
    //stitcher.setRegistrationResol(1.0);
    //stitcher.setCompositingResol(0.1);
    //stitcher.setFeaturesFinder(new cv::detail::OrbFeaturesFinder(cv::Size(3,1),300));


    stitcher.setWarper(new cv::CylindricalWarper());
    stitcher.setWaveCorrection(false);
    stitcher.setSeamEstimationResol(0.001);
    stitcher.setPanoConfidenceThresh(0.1);

    stitcher.setSeamFinder(new cv::detail::GraphCutSeamFinder(cv::detail::GraphCutSeamFinderBase::COST_COLOR_GRAD));
    //stitcher.setSeamFinder(new cv::detail::NoSeamFinder());
    //stitcher.setBlender(cv::detail::Blender::createDefault(cv::detail::Blender::NO, true));
    //stitcher.setExposureCompensator(cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::NO));
    stitcher.setExposureCompensator(new cv::detail::NoExposureCompensator());


    cv::Mat pano_image;
    cv::Stitcher::Status status = stitcher.stitch(images, pano_image);

    while (ros::ok())
    {
        images.clear();
        images.push_back(left_image);
        images.push_back(right_image);

        ros::spinOnce();

        ROS_INFO_STREAM(status << ", " << cv::Stitcher::OK);

        if (status != cv::Stitcher::OK)
        {
            cv::Stitcher::Status status = stitcher.stitch(images, pano_image);
        }
        else
        {
           stitcher.composePanorama(images, pano_image);
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pano_image).toImageMsg();
        pub.publish(msg);

        rate.sleep();
    }

    return 0;
}
