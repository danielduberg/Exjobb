#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <string>

#include <algorithm>

#include <collision_avoidance/point.h>

#include <exjobb_msgs/Control.h>
#include <exjobb_msgs/SensorReadings.h>
#include <collision_avoidance/point.h>

#include <collision_avoidance/obstacle_restriction_method.h>
#include <collision_avoidance/basic.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

# define M_PI           3.14159265358979323846  /* pi */

namespace collision_avoidance
{

class CANodelet : public nodelet::Nodelet
{
public:
    CANodelet() {}

    ~CANodelet() {}

private:
    // Obstacles
    std::vector<Point> obstacles_;

    // Current pose
    geometry_msgs::PoseStamped current_pose_;

    // Current velocity
    float current_direction_, current_speed_;

    // Obstacle-restriction Method
    ORM * orm_;

    // Basic collision avoidance
    Basic * basic_;

    // Subscriptions
    ros::Subscriber sensor_readings_sub_, collision_avoidance_sub_, current_pose_sub_, current_velocity_sub_;

    // Publications
    ros::Publisher collision_free_control_pub_;

    virtual void onInit();

    void sensorReadingsCallback(const exjobb_msgs::SensorReadings::ConstPtr & msg);

    void collisionAvoidanceCallback(const exjobb_msgs::Control::ConstPtr & msg);

    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);

    void currentVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr & msg);
};

void CANodelet::onInit()
{
    ros::NodeHandle nh(getNodeHandle());
    ros::NodeHandle priv_nh(getPrivateNodeHandle());

    sensor_readings_sub_ = nh.subscribe("/sensor_readings", 10, &CANodelet::sensorReadingsCallback, this);
    collision_avoidance_sub_ = nh.subscribe("/control", 10, &CANodelet::collisionAvoidanceCallback, this);
    current_pose_sub_ = nh.subscribe("mavros/local_position/pose", 10, &CANodelet::currentPoseCallback, this);
    current_velocity_sub_ = nh.subscribe("/mavros/local_position/velocity", 10, &CANodelet::currentVelocityCallback, this);

    collision_free_control_pub_ = nh.advertise<exjobb_msgs::Control>("collision_free_control", 10);

    float radius, security_distance, epsilon;
    priv_nh.param<float>("radius", radius, 0.25);
    priv_nh.param<float>("security_distance", security_distance, 0.1);
    priv_nh.param<float>("epsilon", epsilon, 0.1);

    orm_ = new ORM(radius, security_distance, epsilon);
    basic_ = new Basic(radius, security_distance);
}

void CANodelet::sensorReadingsCallback(const exjobb_msgs::SensorReadings::ConstPtr & msg)
{
    std::vector<Point> newObstacles;    // TODO: Init with correct size?

    for (size_t i = 0; i < msg->x.size(); i++)
    {
        Point p;
        p.x = msg->x[i];
        p.y = msg->y[i];
        newObstacles.push_back(p);
    }

    obstacles_ = newObstacles;
}

void CANodelet::collisionAvoidanceCallback(const exjobb_msgs::Control::ConstPtr & msg)
{
    //ROS_ERROR_STREAM("Current speed: " << current_speed_);

    exjobb_msgs::Control collisionFreeControl = *msg;

    Point current;

    current.x = current_speed_ * std::cos(current_direction_ * M_PI / 180.0);
    current.y = current_speed_ * std::sin(current_direction_ * M_PI / 180.0);

    float ab = 1000.0;
    float T = 0.1;

    std::vector<Point> obstacles;

    for (size_t i = 0; i < obstacles_.size(); i++)
    {
        if (obstacles_[i].x == 0 && obstacles_[i].y == 0)
        {
            obstacles.push_back(obstacles_[i]);
            continue;
        }

        float dobs = Point::getDistance(obstacles_[i]);

        float deff = ab * (T * T) * (std::sqrt(1.0 + ((2 * dobs) / (ab * (T * T)))) - 1.0);

        Point temp;
        temp.x = deff * std::cos(current_direction_ * M_PI / 180.0);
        temp.y = deff * std::sin(current_direction_ * M_PI / 180.0);

        Point point;
        point.x = deff * std::cos(Point::getDirection(obstacles_[i]));
        point.y = deff * std::sin(Point::getDirection(obstacles_[i]));

        // 10 hz
        //point.x = obstacles_[i].x - (temp.x);
        //point.y = obstacles_[i].y - (temp.y);
        obstacles.push_back(point);
    }

    // Call ORM
    orm_->avoidCollision(&collisionFreeControl, obstacles);

    // Call validation
    basic_->avoidCollision(&collisionFreeControl, obstacles, current_direction_, current_speed_);

    // Publish

    //ROS_ERROR_STREAM(msg->go_direction << ", " << msg->go_magnitude << "; " << collisionFreeControl.go_direction << ", " << collisionFreeControl.go_magnitude);

    collision_free_control_pub_.publish(collisionFreeControl);
}

void CANodelet::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    current_pose_ = *msg;
}

void CANodelet::currentVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
    double roll, pitch, yaw;
    tf2::Quaternion q(current_pose_.pose.orientation.x, current_pose_.pose.orientation.y, current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getEulerYPR(yaw, pitch, roll);

    Point point;
    point.x = msg->twist.linear.x;
    point.y = msg->twist.linear.y;

    float direction, magnitude;
    Point::getVectorFromPointDegrees(point, direction, magnitude);

    current_speed_ = magnitude;

    current_direction_ = direction - (yaw * 180.0 / M_PI);

    if (current_direction_ < 0)
    {
        current_direction_ += 360;
    }
    else if (current_direction_ > 360)
    {
        current_direction_ -= 360;
    }

    //ROS_ERROR_STREAM("Direction: " << current_direction_ << ", speed: " << current_speed_);
}

// End namespace
}

// Register this plugin with pluginlib.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(collision_avoidance, CA, collision_avoidance::CANodelet, nodelet::Nodelet);
