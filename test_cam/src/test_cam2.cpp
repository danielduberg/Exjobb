#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

#include "std_msgs/UInt16.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void cameraCallback(const stereo_msgs::DisparityImage::ConstPtr & msg) {
    const cv::Mat_<float> dmat(msg->image.height,
                               msg->image.width,
                               (float*)&msg->image.data[0],
                               msg->image.step);

    int numHistStuff = msg->image.width;
    float hist[numHistStuff];
    std::fill_n(hist, numHistStuff, 0);

    for (size_t x = 0; x < msg->image.width; x++) {
        for (size_t y = (msg->image.height / 2) - 5; y < (msg->image.height / 2) + 10; y++) {
            int index = x / (msg->image.width / numHistStuff);

            float depth = (msg->f * msg->T) / dmat(y, x);

            if (depth > 0.1 && depth < 5 && (depth < hist[index] || hist[index] == 0)) {
                hist[index] = depth;
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.width = numHistStuff;
    cloud.height = 1;
    cloud.is_dense = false;

    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < numHistStuff; i++) {
        cloud.points[i].z = 0;
        cloud.points[i].x = (i - (numHistStuff / 2.0)) * hist[i] / msg->f;
        cloud.points[i].y = hist[i];
    }

    cloud.header.frame_id = "cloud";

    pub.publish(cloud);

    //rostopic pub -1 /rumble joy_feedback_ros/Rumble 18000 18000
    //rostopic pub /play std_msgs/UInt16 1
    /*
    for (int i = 0; i < numHistStuff; i++) {
        std::cout.precision(3);

        if (hist[i] < 1.2 && hist[i] > 1.0) {
            std_msgs::UInt16 durr;
            durr.data = rand() % 2; // 0 or 1
            pub.publish(durr);
            std::cout << hist[i] << ", ";
        } else {
            std::cout << "0.000,";
        }
    }
    std::cout << std::endl;
    */
    //std::cout << (msg->f * msg->T) / dmat(160, 120) << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_cam2");

    ros::NodeHandle nh;

    //pub = nh.advertise<std_msgs::UInt16>("play", 1000);
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("cloud", 1000);

    ros::Subscriber sub = nh.subscribe("/stereo/disparity", 1000, cameraCallback);

    ros::spin();

    return 0;
}
