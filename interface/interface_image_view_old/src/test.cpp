#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    std::string path = ros::package::getPath("interface_image_view") + "/";

    cv::Mat left_image = cv::imread(path + "left1.png");
    cv::Mat right_image = cv::imread(path + "right1.png");

    std::vector<cv::Mat> images;
    images.push_back(left_image);
    images.push_back(right_image);


    image_transport::Publisher pub = it.advertise("pano", 10);

    cv::Stitcher stitcher = cv::Stitcher::createDefault(true);
    stitcher.setWarper(new cv::CylindricalWarper());
    stitcher.setWaveCorrection(false);
    stitcher.setSeamEstimationResol(0.001);
    stitcher.setPanoConfidenceThresh(0.1);

    //stitcher.setSeamFinder(new cv::detail::GraphCutSeamFinder(cv::detail::GraphCutSeamFinderBase::COST_COLOR_GRAD));
    stitcher.setSeamFinder(new cv::detail::NoSeamFinder());
    stitcher.setBlender(cv::detail::Blender::createDefault(cv::detail::Blender::NO, true));
    //stitcher.setExposureCompensator(cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::NO));
    stitcher.setExposureCompensator(new cv::detail::NoExposureCompensator());

    cv::Mat pano_image;
    stitcher.stitch(images, pano_image);

    ros::Rate rate(100);
    while (ros::ok())
    {
        stitcher.composePanorama(images, pano_image);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pano_image).toImageMsg();
        pub.publish(msg);

        rate.sleep();
    }

    return 0;
}
