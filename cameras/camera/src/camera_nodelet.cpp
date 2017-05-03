#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <image_transport/image_transport.h>

#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
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
    // Camera infos
    std::vector<camera_info_manager::CameraInfoManager *> camera_infos;

    // Subscriptions
    std::vector<image_transport::Subscriber> image_subs;
    std::vector<ros::Subscriber> disparity_subs;

    // Publications
    std::vector<image_transport::Publisher> image_pubs;
    std::vector<ros::Publisher> disparity_pubs;
    std::vector<ros::Publisher> camera_info_pubs;

    virtual void onInit();

    void imageCallback(const sensor_msgs::Image::ConstPtr & msg, const image_transport::Publisher & pub, camera_info_manager::CameraInfoManager * cim, const ros::Publisher & camera_info_pub);

    void disparityCallback(const stereo_msgs::DisparityImage::ConstPtr & msg, const ros::Publisher & pub);
};

void CameraNodelet::onInit()
{
    ros::NodeHandle nh(getNodeHandle());
    ros::NodeHandle priv_nh(getPrivateNodeHandle());

    std::vector<std::string> sub_topics;
    priv_nh.getParam("sub_topics", sub_topics);

    std::vector<std::string> camera_info_urls;
    priv_nh.getParam("camera_info_urls", camera_info_urls);

    camera_infos.reserve(2 * camera_info_urls.size());
    camera_info_pubs.reserve(2 * camera_info_urls.size());

    image_subs.reserve(2 * sub_topics.size());
    image_pubs.reserve(2 * sub_topics.size());

    disparity_subs.reserve(sub_topics.size());
    disparity_pubs.reserve(sub_topics.size());

    image_transport::ImageTransport it(nh);

    for (size_t i = 0; i < sub_topics.size(); i++)
    {
        camera_infos.push_back(new camera_info_manager::CameraInfoManager(ros::NodeHandle(sub_topics[i] + "/flipped/right"), "camera", camera_info_urls[i] + "right.yaml"));
        camera_info_pubs.push_back(nh.advertise<sensor_msgs::CameraInfo>(sub_topics[i] + "/flipped/right/camera_info", 10));

        image_pubs.push_back(it.advertise(sub_topics[i] + "/flipped/right/image_rect", 10));
        image_subs.push_back(it.subscribe(sub_topics[i] + "/left/image_rect", 10, boost::bind(&CameraNodelet::imageCallback, this, _1, image_pubs.back(), camera_infos.back(), camera_info_pubs.back())));


        camera_infos.push_back(new camera_info_manager::CameraInfoManager(ros::NodeHandle(sub_topics[i] + "/flipped/left"), "camera", camera_info_urls[i] + "left.yaml"));
        camera_info_pubs.push_back(nh.advertise<sensor_msgs::CameraInfo>(sub_topics[i] + "/flipped/left/camera_info", 10));

        image_pubs.push_back(it.advertise(sub_topics[i] + "/flipped/left/image_rect", 10));
        image_subs.push_back(it.subscribe(sub_topics[i] + "/right/image_rect", 10, boost::bind(&CameraNodelet::imageCallback, this, _1, image_pubs.back(), camera_infos.back(), camera_info_pubs.back())));

        disparity_pubs.push_back(nh.advertise<stereo_msgs::DisparityImage>(sub_topics[i] + "/disparity_complement", 10));
        disparity_subs.push_back(nh.subscribe<stereo_msgs::DisparityImage>(sub_topics[i] + "/flipped/disparity", 10, boost::bind(&CameraNodelet::disparityCallback, this, _1, disparity_pubs.back())));
    }
}

void CameraNodelet::imageCallback(const sensor_msgs::Image::ConstPtr & msg, const image_transport::Publisher & pub, camera_info_manager::CameraInfoManager * cim, const ros::Publisher & camera_info_pub)
{
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

    cv::Mat image_flipped;
    cv::flip(image, image_flipped, 1);

    sensor_msgs::Image::ConstPtr out = cv_bridge::CvImage(msg->header, "bgr8", image_flipped).toImageMsg();

    pub.publish(out);

    sensor_msgs::CameraInfo camera_info = cim->getCameraInfo();
    camera_info.header = msg->header;
    camera_info_pub.publish(camera_info);
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

    sensor_msgs::Image & disp_image = out_msg.image;

    disp_image.width = dmat_flipped.size().width;
    disp_image.height = dmat_flipped.size().height;
    disp_image.encoding = msg->image.encoding;
    disp_image.step = msg->image.step;
    disp_image.data.resize(disp_image.step * disp_image.height);

    cv::Mat_<float> dmat_final(disp_image.height, disp_image.width, (float*)&disp_image.data[0], disp_image.step);

    dmat_flipped.convertTo(dmat_final, dmat.type());

    pub.publish(out_msg);
}

// End namespace
}

// Register this plugin with pluginlib.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(camera, Camera, camera::CameraNodelet, nodelet::Nodelet);
