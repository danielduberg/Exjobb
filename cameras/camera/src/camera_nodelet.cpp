#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <image_transport/image_transport.h>

#include <stereo_msgs/DisparityImage.h>

#include <cv_bridge/cv_bridge.h>

#include <string>

#include <algorithm>

# define M_PI           3.14159265358979323846  /* pi */

namespace camera
{

class CameraNodelet : public nodelet::Nodelet
{
public:
    CameraNodelet() {}

    ~CameraNodelet() {}

private:
    // Subscriptions
    std::vector<image_transport::Subscriber> image_subs;
    std::vector<ros::Subscriber> disparity_subs;
    //ros::Subscriber sensor_readings_sub_, collision_avoidance_sub_, current_pose_sub_, current_velocity_sub_;

    // Publications
    std::vector<image_transport::Publisher> image_pubs;
    std::vector<ros::Publisher> disparity_pubs;
    //ros::Publisher collision_free_control_pub_;

    virtual void onInit();

    void imageCallback(const sensor_msgs::Image::ConstPtr & msg, const image_transport::Publisher & pub);
    //void sensorReadingsCallback(const exjobb_msgs::SensorReadings::ConstPtr & msg);

    void disparityCallback(const stereo_msgs::DisparityImage::ConstPtr & msg, const ros::Publisher & pub);
};

void CameraNodelet::onInit()
{
    ros::NodeHandle nh(getNodeHandle());
    ros::NodeHandle priv_nh(getPrivateNodeHandle());

    std::vector<std::string> sub_topics;
    priv_nh.getParam("sub_topics", sub_topics);

    image_subs.reserve(2 * sub_topics.size());
    image_pubs.reserve(2 * sub_topics.size());

    disparity_subs.reserve(sub_topics.size());
    disparity_pubs.reserve(sub_topics.size());

    image_transport::ImageTransport it(nh);

    for (size_t i = 0; i < sub_topics.size(); i++)
    {
        image_pubs.push_back(it.advertise(sub_topics[i] + "/reversed/raw/right/image_rect_color", 10));
        image_subs.push_back(it.subscribe(sub_topics[i] + "/left/image_rect_color", 10, boost::bind(&CameraNodelet::imageCallback, this, _1, image_pubs.back())));

        image_pubs.push_back(it.advertise(sub_topics[i] + "/reversed/raw/left/image_rect_color", 10));
        image_subs.push_back(it.subscribe(sub_topics[i] + "/right/image_rect_color", 10, boost::bind(&CameraNodelet::imageCallback, this, _1, image_pubs.back())));

        disparity_pubs.push_back(nh.advertise<stereo_msgs::DisparityImage>(sub_topics[i] + "/reversed/disparity", 10));
        disparity_subs.push_back(nh.subscribe<stereo_msgs::DisparityImage>(sub_topics[i] + "/reversed/raw/disparity", 10, boost::bind(&CameraNodelet::disparityCallback, this, _1, disparity_pubs.back())));
    }

    //sensor_readings_sub_ = nh.subscribe("/sensor_readings", 10, &CANodelet::sensorReadingsCallback, this);

    //collision_free_control_pub_ = nh.advertise<exjobb_msgs::Control>("collision_free_control", 10);

    float radius, security_distance, epsilon;
    priv_nh.param<float>("radius", radius, 0.25);
    priv_nh.param<float>("security_distance", security_distance, 0.1);
    priv_nh.param<float>("epsilon", epsilon, 0.1);
}

void CameraNodelet::imageCallback(const sensor_msgs::Image::ConstPtr & msg, const image_transport::Publisher & pub)
{

}

void CameraNodelet::disparityCallback(const stereo_msgs::DisparityImage::ConstPtr & msg, const ros::Publisher & pub)
{
    stereo_msgs::DisparityImage out_msg = *msg;

    const cv::Mat_<float> dmat(msg->image.height,
                               msg->image.width,
                               (float*)&msg->image.data[0],
                               msg->image.step);

    cv::Mat_<float> dmat_flipped;

    cv::flip(dmat, dmat_flipped, 1);


}

// End namespace
}

// Register this plugin with pluginlib.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(camera, Camera, camera::CameraNodelet, nodelet::Nodelet);
