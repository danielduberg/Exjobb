#include <ros/ros.h>

#include <exjobb_msgs/Control.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

# define M_PI           3.14159265358979323846  /* pi */

mavros_msgs::State current_state;

geometry_msgs::PoseStamped current_pose;

ros::Publisher local_pos_pub;
ros::Publisher cmd_vel_pub;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

float altitude = 1;

void stateCallback(const mavros_msgs::State::ConstPtr & msg)
{
    current_state = *msg;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    current_pose = *msg;
}

void arm_disarm(bool arm)
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;

    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        if (arm)
        {
            ROS_INFO_STREAM_THROTTLE(1, "Vehicle armed");
        }
        else
        {
            ROS_INFO_STREAM_THROTTLE(1, "Vehicle disarmed");
        }
    }
    else
    {
        if (arm)
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Could not arm vehicle");
        }
        else
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Could not disarm vehicle");
        }
    }
}

void land()
{
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "AUTO.LAND";   // Makes it land

    if (set_mode_client.call(set_mode) && set_mode.response.success)
    {
        ROS_INFO_STREAM_THROTTLE(1, "Starting to land");
    }
    else
    {
        ROS_ERROR_STREAM_THROTTLE(1, "Could not land");
    }
}

void lift()
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = altitude;

    // Send a few vel before starting
    for (int i = 0; ros::ok() && i < 100; i++)
    {
        local_pos_pub.publish(pose);
    }

    if (current_state.mode != "OFFBOARD")
    {
        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = "OFFBOARD";

        if (set_mode_client.call(set_mode) && set_mode.response.success)
        {
            ROS_INFO_STREAM_THROTTLE(1, "Offboard mode enabled");
        }
        else
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Could not enable offboard mode");
            return;
        }
    }

    local_pos_pub.publish(pose);
}

void fly(float go_direction, float go_magnitude, float rotate, bool turn_to_where_going)
{
    double roll, pitch, yaw;
    tf2::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getEulerYPR(yaw, pitch, roll);

    // Move!
    geometry_msgs::TwistStamped twist;

    // Linear
    twist.twist.linear.x = go_magnitude * std::cos((go_direction * M_PI / 180.0) + yaw);
    twist.twist.linear.y = go_magnitude * std::sin((go_direction * M_PI / 180.0) + yaw);

    if (current_pose.pose.position.z < altitude)
    {
        if (altitude - current_pose.pose.position.z < 0.1)
        {
            twist.twist.linear.z = 0.15;
        }
        else
        {
            twist.twist.linear.z = 0.3;
        }

        twist.twist.linear.z = std::max(0.0, 0.4 * (1.0 - (current_pose.pose.position.z / altitude)));
    }

    if (turn_to_where_going && rotate == 0 && go_magnitude != 0)
    {
        float yaw_degrees = yaw * 180.0 / M_PI; // Yaw is current heading

        if (yaw_degrees < 0)
        {
            yaw_degrees += 360;
        }
        else if (yaw_degrees >= 360)
        {
            yaw_degrees -= 360;
        }

        float diff = go_direction - yaw_degrees;

        if (diff > 180)
        {
            diff -= 360;
        }
        if (diff < -180)
        {
            diff += 360;
        }

        rotate = (diff > 0) - (diff < 0);   // -1 when diff < 0, 1 when diff > 0, 0 otherwise
    }

    // Angular
    // -0.75
    twist.twist.angular.z = 0.5 * rotate;

    cmd_vel_pub.publish(twist);
}

void controlCallback(const exjobb_msgs::Control::ConstPtr & msg)
{
    if (msg->disarm)
    {
        arm_disarm(false);
    }
    else if (msg->arm)
    {
        arm_disarm(true);
    }
    else if (msg->land)
    {
        land();
    }
    else if (msg->lift)
    {
        lift();
    }
    else //if (current_state.armed && current_state.mode == "OFFBOARD")
    {
        fly(msg->go_direction, msg->go_magnitude, msg->rotate, msg->turn_to_where_going);
    }
    //else if (msg->go_direction != 0 || msg->go_magnitude != 0 || msg->rotate != 0)
    //{
    //    ROS_ERROR_STREAM_THROTTLE(1, "The drone has to be armed and in OFFBOARD mode to fly");
    //}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    nh_priv.param<float>("altitude", altitude, 1.1);
    std::string control_topic;
    nh_priv.getParam("control_topic", control_topic);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, poseCallback);

    ros::Subscriber control_sub = nh.subscribe<exjobb_msgs::Control>(control_topic, 10, controlCallback);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");


    float frequency;
    nh_priv.param<float>("frequency", frequency, 20);
    ros::Rate rate(frequency);

    ROS_INFO_STREAM("Wanted altitude: " << altitude << " m, Control topic: '" << control_topic << "', Frequency: " << frequency);

    // wait for FCU connection
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    return 0;
}
