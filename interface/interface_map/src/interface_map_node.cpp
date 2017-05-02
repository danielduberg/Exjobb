#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_publisher.h>

#include <sensor_msgs/Image.h>

#include <exjobb_msgs/SensorReadings.h>

#include <exjobb_msgs/View.h>

#include <exjobb_msgs/Control.h>

#include <exjobb_msgs/ORM.h>

#define PI 3.14159265

image_transport::Publisher pub_;

float height_, width_, obstacle_size_, control_size_, target_size_, pixels_per_meter_, robot_length_, robot_width_, line_thickness_, lines_per_meter_, sight_cone_range_, min_distance_between_obstacles_;

float look_direction_ = 90;
float look_fov_ = 120;

cv::Scalar background_color_, obstacle_color_, sight_cone_color_, control_color_, target_color_, robot_color_, meter_line_color_, ORM_goal_color_, ORM_left_bound_color_, ORM_right_bound_color_, ORM_radius_color_, ORM_radius_security_distance_color_;

cv::Point2f control_(0, 1);
cv::Point2f target_(0.3, 1);

cv::Point2f ORM_goal_(0, 0);
cv::Point2f ORM_left_bound_(0, 0);
cv::Point2f ORM_right_bound_(0, 0);

bool ORM_debug_;

float ORM_radius_ = 0, ORM_security_distance_ = 0;
float ORM_goal_size_, ORM_left_bound_size_, ORM_right_bound_size_, ORM_radius_size_;
std::vector<cv::Point2f> ORM_left_bound_points_, ORM_right_bound_points_;

void controlCallback(const exjobb_msgs::Control::ConstPtr & msg)
{
    // Reversed because OpenCV...
    control_.y = msg->go_magnitude * std::cos(msg->go_direction * PI / 180.0);
    control_.x = msg->go_magnitude * std::sin(msg->go_direction * PI / 180.0);
}

void targetCallback(const exjobb_msgs::Control::ConstPtr & msg)
{
    // Reversed because OpenCV...
    target_.y = msg->go_magnitude * std::cos(msg->go_direction * PI / 180.0);
    target_.x = msg->go_magnitude * std::sin(msg->go_direction * PI / 180.0);
}

void ORMCallback(const exjobb_msgs::ORM::ConstPtr & msg)
{
    ORM_radius_ = msg->radius;
    ORM_security_distance_ = msg->security_distance;

    ROS_ERROR_STREAM(msg->goal_direction << ", " << msg->left_bound << ", " << msg->right_bound);
    // Reversed because OpenCV...
    ORM_goal_.y = std::cos(msg->goal_direction * PI / 180.0);
    ORM_goal_.x = std::sin(msg->goal_direction * PI / 180.0);

    if (msg->left_bound == -1)
    {
        ORM_left_bound_.y = 0;
        ORM_left_bound_.x = 0;
    }
    else
    {
        ORM_left_bound_.y = std::cos(msg->left_bound * PI / 180.0);
        ORM_left_bound_.x = std::sin(msg->left_bound * PI / 180.0);
    }

    if (msg->right_bound == -1)
    {
        ORM_right_bound_.y = 0;
        ORM_right_bound_.x = 0;
    }
    else
    {
        ORM_right_bound_.y = std::cos(msg->right_bound * PI / 180.0);
        ORM_right_bound_.x = std::sin(msg->right_bound * PI / 180.0);
    }

    ORM_left_bound_points_.clear();
    ORM_right_bound_points_.clear();

    cv::Point2f center(width_ / 2.0, height_ / 2.0);

    for (size_t i = 0; i < msg->left_bound_points_x.size(); i++)
    {
        ORM_left_bound_points_.push_back(center - cv::Point2f(msg->left_bound_points_y[i] * pixels_per_meter_, msg->left_bound_points_x[i] * pixels_per_meter_));
    }

    for (size_t i = 0; i < msg->right_bound_points_x.size(); i++)
    {
        ORM_right_bound_points_.push_back(center - cv::Point2f(msg->right_bound_points_y[i] * pixels_per_meter_, msg->right_bound_points_x[i] * pixels_per_meter_));
    }
}

void viewCallback(const exjobb_msgs::View::ConstPtr & msg) {
    look_direction_ = msg->direction;
    look_fov_ = msg->fov;
}

float getDistance(cv::Point2f p_1, cv::Point2f p_2) {
    return std::sqrt(((p_1.x - p_2.x) * (p_1.x - p_2.x)) + ((p_1.y - p_2.y) * (p_1.y - p_2.y)));
}

void sensorReadingCallback(const exjobb_msgs::SensorReadings::ConstPtr & msg) {
    cv::Mat map(height_, width_, CV_8UC3, background_color_);

    cv::Point2f center(width_ / 2.0, height_ / 2.0);

    // Sight cone
    cv::ellipse(map, center, cv::Size(sight_cone_range_ * pixels_per_meter_, sight_cone_range_ * pixels_per_meter_), 0, -look_direction_ - (look_fov_ / 2.0), -look_direction_ + (look_fov_ / 2.0), sight_cone_color_, CV_FILLED);

    if (lines_per_meter_ > 0) {
        // Draw horisontal lines each half meter
        for (int i = 0; (height_ / 2) - i > 0; i += (pixels_per_meter_ / lines_per_meter_)) {
            cv::line(map, cv::Point(0, (height_ / 2) + i), cv::Point(width_, (height_ / 2) + i), meter_line_color_, line_thickness_);
            cv::line(map, cv::Point(0, (height_ / 2) - i), cv::Point(width_, (height_ / 2) - i), meter_line_color_, line_thickness_);
        }

        // Draw vertical lines each half meter
        for (int i = 0; (width_ / 2) - i > 0; i += (pixels_per_meter_ / lines_per_meter_)) {
            cv::line(map, cv::Point((width_ / 2) + i, 0), cv::Point((width_ / 2) + i, height_), meter_line_color_, line_thickness_);
            cv::line(map, cv::Point((width_ / 2) - i, 0), cv::Point((width_ / 2) - i, height_), meter_line_color_, line_thickness_);
        }
    }

    // Put out obstacles
    cv::Point2f first(0, 0);
    cv::Point2f last(0, 0);
    for (size_t i = 0; i < msg->x.size(); i++) {
        if (msg->x[i] == 0 && msg->y[i] == 0) {
            continue;
        }

        if (first.x == 0 && first.y == 0) {
            first.x = msg->y[i] * pixels_per_meter_;
            first.y = msg->x[i] * pixels_per_meter_;
        }

        cv::Point2f current(msg->y[i] * pixels_per_meter_, msg->x[i] * pixels_per_meter_);

        current = center - current;

        if (last.x != 0 || last.y != 0) {
            if (getDistance(last, current) < min_distance_between_obstacles_ * pixels_per_meter_) {
                // Same object!
                cv::line(map, last, current, obstacle_color_, obstacle_size_);
                last = current;
                continue;
            }
        }

        cv::circle(map, current, round(obstacle_size_ / 2.0), obstacle_color_, CV_FILLED);

        last = current;
    }

    if ((first.x != 0 || first.y != 0) && (last.x != 0 || last.y != 0)) {
        if (getDistance(first, last) < min_distance_between_obstacles_ * pixels_per_meter_) {
            cv::line(map, first, last, obstacle_color_, obstacle_size_);
        }
    }

    // Put the control and target point out!
    cv::circle(map, center - (control_ * pixels_per_meter_), round(control_size_ / 2.0), control_color_, CV_FILLED);
    cv::circle(map, center - (target_ * pixels_per_meter_), round(target_size_ / 2.0), target_color_, CV_FILLED);

    // Put the ORM points out!
    cv::circle(map, center, round(ORM_radius_ * pixels_per_meter_), ORM_radius_color_, ORM_radius_size_);
    if (ORM_debug_)
    {
        cv::circle(map, center - (ORM_goal_ * pixels_per_meter_), round(ORM_goal_size_ / 2.0), ORM_goal_color_, CV_FILLED);
        cv::circle(map, center - (ORM_left_bound_ * pixels_per_meter_), round(ORM_left_bound_size_ / 2.0), ORM_left_bound_color_, CV_FILLED);
        cv::circle(map, center - (ORM_right_bound_ * pixels_per_meter_), round(ORM_right_bound_size_ / 2.0), ORM_right_bound_color_, CV_FILLED);

        cv::circle(map, center, round((ORM_radius_ + ORM_security_distance_) * pixels_per_meter_), ORM_radius_security_distance_color_, ORM_radius_size_);

        for (size_t i = 0; i < ORM_left_bound_points_.size(); i++)
        {
            cv::circle(map, ORM_left_bound_points_[i], round(ORM_left_bound_size_ / 2.0), ORM_left_bound_color_, ORM_radius_size_);
        }

        for (size_t i = 0; i < ORM_right_bound_points_.size(); i++)
        {
            cv::circle(map, ORM_right_bound_points_[i], round(ORM_right_bound_size_ / 2.0), ORM_right_bound_color_, ORM_radius_size_);
        }
    }

    // Draw the robot!
    //cv::Point2f robot(robot_width_ * pixels_per_meter_ / 2.0, robot_length_ * pixels_per_meter_ / 2.0);
    //cv::rectangle(map, center + robot, center - robot, robot_color_, CV_FILLED);
    cv::Point2f robot(robot_width_ * pixels_per_meter_ / 4.0, robot_length_ * pixels_per_meter_ / 4.0);
    cv::rectangle(map, center + robot, center - robot, robot_color_, CV_FILLED);
    cv::Point2f propeller = 1.25 * robot;
    cv::circle(map, center + propeller, robot_width_ * pixels_per_meter_ / 6.0, robot_color_, CV_FILLED); // cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(map, center - propeller, robot_width_ * pixels_per_meter_ / 6.0, robot_color_, CV_FILLED); // cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(map, center + cv::Point2f(-propeller.x, propeller.y), robot_width_ * pixels_per_meter_ / 6.0, robot_color_, CV_FILLED); // cv::Scalar(0, 255, 0), CV_FILLED);
    cv::circle(map, center + cv::Point2f(propeller.x, -propeller.y), robot_width_ * pixels_per_meter_ / 6.0, robot_color_, CV_FILLED); // cv::Scalar(0, 255, 0), CV_FILLED);

    sensor_msgs::Image::ConstPtr out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", map).toImageMsg();

    pub_.publish(out);
}

void initParams(ros::NodeHandle & nh) {
    nh.param<float>("height", height_, 100);
    nh.param<float>("width", width_, 100);

    nh.param<float>("pixels_per_meter", pixels_per_meter_, 10);

    nh.param<float>("obstacle_size", obstacle_size_, 5);
    nh.param<float>("control_size", control_size_, 7);
    nh.param<float>("target_size", target_size_, 7);

    nh.param<float>("robot_length", robot_length_, 0.4);
    nh.param<float>("robot_width", robot_width_, 0.4);

    nh.param<float>("line_thickness", line_thickness_, 2);
    nh.param<float>("lines_per_meter", lines_per_meter_, 0);

    nh.param<float>("sight_cone_range", sight_cone_range_, 1);

    nh.param<float>("min_distance_between_obstacles", min_distance_between_obstacles_, 0.1);

    nh.param<bool>("orm_debug", ORM_debug_, false);
    nh.param<float>("ORM_goal_size", ORM_goal_size_, 7);
    nh.param<float>("ORM_left_bound_size", ORM_left_bound_size_, 7);
    nh.param<float>("ORM_right_bound_size", ORM_right_bound_size_, 7);
    nh.param<float>("ORM_radius_size", ORM_radius_size_, 5);

    std::vector<float> color;
    nh.getParam("background_color", color);
    background_color_ = cv::Scalar(color[0], color[1], color[2]);

    nh.getParam("obstacle_color", color);
    obstacle_color_ = cv::Scalar(color[0], color[1], color[2]);

    nh.getParam("sight_cone_color", color);
    sight_cone_color_ = cv::Scalar(color[0], color[1], color[2]);

    nh.getParam("control_color", color);
    control_color_ = cv::Scalar(color[0], color[1], color[2]);

    nh.getParam("target_color", color);
    target_color_ = cv::Scalar(color[0], color[1], color[2]);

    nh.getParam("robot_color", color);
    robot_color_ = cv::Scalar(color[0], color[1], color[2]);

    nh.getParam("meter_line_color", color);
    meter_line_color_ = cv::Scalar(color[0], color[1], color[2]);

    nh.getParam("ORM_goal_color", color);
    ORM_goal_color_= cv::Scalar(color[0], color[1], color[2]);

    nh.getParam("ORM_left_bound_color", color);
    ORM_left_bound_color_= cv::Scalar(color[0], color[1], color[2]);

    nh.getParam("ORM_right_bound_color", color);
    ORM_right_bound_color_= cv::Scalar(color[0], color[1], color[2]);

    nh.getParam("ORM_radius_color", color);
    ORM_radius_color_= cv::Scalar(color[0], color[1], color[2]);

    nh.getParam("ORM_radius_security_distance_color", color);
    ORM_radius_security_distance_color_= cv::Scalar(color[0], color[1], color[2]);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "interface_map");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    initParams(nh_priv);

    image_transport::ImageTransport it(nh);

    std::string pub_topic;
    nh_priv.param<std::string>("pub_topic", pub_topic, "map");

    pub_ = it.advertise(pub_topic, 1000);

    ros::Subscriber sub = nh.subscribe("/sensor_readings", 1000, sensorReadingCallback);

    ros::Subscriber view_sub = nh.subscribe("/view", 1000, viewCallback);

    ros::Subscriber control_sub = nh.subscribe("/control", 1000, controlCallback);
    ros::Subscriber target_sub = nh.subscribe("/collision_free_control", 1000, targetCallback);

    ros::Subscriber orm_sub = nh.subscribe("/orm", 1000, ORMCallback);

    ros::spin();

    return 0;
}
