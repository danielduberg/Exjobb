#include <ros/ros.h>

#include "sensor_msgs/Image.h"

#include <opencv2/stitching/stitcher.hpp>

class Image {
public:
	sensor_msgs::Image image_;
	
    const std::string topic_;
	
    const float look_direction_, fov_;
    
    cv::Stitcher stitcher_;
    
    bool stitcher_ready_;
    
    Image(std::string topic, float direction, float fov, std::string image_path);
    
    void configureStitcher();
    
    bool stitch(std::vector<cv::Mat> & images, cv::Mat * pano_image = NULL);

	void callback(const sensor_msgs::Image::ConstPtr & msg);
};
