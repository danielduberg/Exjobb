#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

static const std::string OPENCV_WINDOW = "Image window";

void cameraCallback(const sensor_msgs::Image::ConstPtr& msg1, const sensor_msgs::Image::ConstPtr& msg2, const sensor_msgs::Image::ConstPtr& msg3) {
	cv::Mat depthImgArray[] = {
		cv_bridge::toCvCopy(msg3, sensor_msgs::image_encodings::TYPE_16UC1)->image,
		cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::TYPE_16UC1)->image,
		cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::TYPE_16UC1)->image
	};

	cv::Mat depthImgFlipped;

	cv::hconcat(depthImgArray, 3, depthImgFlipped);

	cv::Mat depthImg;

	cv::flip(depthImgFlipped, depthImg, 1);

	int numRows = 50;

	int middle = depthImg.rows / 2;

	std::vector<unsigned short *> middleRows;
	for (int i = middle - numRows; i < middle + numRows; i++) {
		middleRows.push_back(depthImg.ptr<unsigned short>(i));
	}

	for (size_t i = 0; i < depthImg.rows; i++) {
		unsigned short * row = depthImg.ptr<unsigned short>(i);
		for (size_t j = 0; j < depthImg.cols; j++) {
			if (row[j] <= 0) {
				row[j] -= 1;	// All that are too far away
			} else {
				if (row[j] < middleRows[0][j]) {
					middleRows[0][j] = row[j];
				}
				row[j] = -1;
			}
		}
	}

	for (size_t i = 1; i < middleRows.size(); i++) {
		for (size_t j = 0; j < depthImg.cols; j++) {
			middleRows[i][j] = middleRows[0][j];
		}
	}

	double min, max;
	cv::minMaxIdx(depthImg, &min, &max);

	cv::Mat adjMap;

	depthImg.convertTo(adjMap, CV_8UC1, 0.2);

	cv::Mat falseColorsMap;
	cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);

	cv::imshow(OPENCV_WINDOW, falseColorsMap);
	cv::waitKey(3);

	//imwrite("./depth.jpg", depthImg->image);
	//ros::shutdown();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_cam");

	ros::NodeHandle n;

	message_filters::Subscriber<sensor_msgs::Image> camera1_sub(n, "/camera1/depth/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> camera2_sub(n, "/camera2/depth/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> camera3_sub(n, "/camera3/depth/image_raw", 1);

	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> > sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>(10), camera1_sub, camera2_sub, camera3_sub);

	//message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync(camera1_sub, camera2_sub, camera3_sub, 10);
	sync.registerCallback(boost::bind(&cameraCallback, _1, _2, _3));
	//ros::Subscriber sub = n.subscribe("/camera1/depth/image_raw", 1000, cameraCallback);

	ros::spin();

	return 0;
}
