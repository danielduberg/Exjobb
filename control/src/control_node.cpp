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

ros::Publisher localPosPub;
ros::Publisher cmdVelPub;

ros::ServiceClient armingClient;
ros::ServiceClient setModeClient;

void stateCallback(const mavros_msgs::State::ConstPtr & msg) {
    current_state = *msg;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg) {
    current_pose = *msg;
}

void controlCallback(const exjobb_msgs::Control::ConstPtr & msg) {
    if (msg->land) {
        // Disarm
        mavros_msgs::CommandBool disarmCmd;
        disarmCmd.request.value = false;

        if (armingClient.call(disarmCmd) && disarmCmd.response.success) {
            ROS_INFO("Vehicle disarmed");
        } else {
            ROS_ERROR("Could not disarm vehicle");
        }
        return;
    }

    if (!current_state.armed || current_state.mode != "OFFBOARD") {
        if (msg->lift) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.z = 2;

            // Send a few vel before starting
            for (int i = 0; ros::ok() && i < 100; i++) {
                localPosPub.publish(pose);
            }

            if (!current_state.armed) {
                // Arm
                mavros_msgs::CommandBool armCmd;
                armCmd.request.value = true;

                if (armingClient.call(armCmd) && armCmd.response.success) {
                    ROS_INFO("Vehicle armed");
                } else {
                    ROS_ERROR("Could not arm vehicle");
                    return;
                }
            }

            if (current_state.mode != "OFFBOARD") {
                mavros_msgs::SetMode setMode;
                setMode.request.custom_mode = "OFFBOARD";

                if (setModeClient.call(setMode) && setMode.response.success) {
                    ROS_INFO("Offboard enabled");
                } else {
                    ROS_ERROR("Could not enable offboard");
                    return;
                }
            }

            localPosPub.publish(pose);

            return;
        } else {
            return;
        }
    }

    double roll, pitch, yaw;
    tf2::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getEulerYPR(yaw, pitch, roll);

    // Move!
    geometry_msgs::TwistStamped twist;

    // Linear
    twist.twist.linear.x = 2 * msg->goMagnitude * std::cos(((msg->goDirection) * M_PI / 180.0) + yaw);
    twist.twist.linear.y = 2 * msg->goMagnitude * std::sin(((msg->goDirection) * M_PI / 180.0) + yaw);

    if (current_pose.pose.position.z < 1) {
        if (1 - current_pose.pose.position.z < 0.1) {
            twist.twist.linear.z = 0.15;
        } else {
            twist.twist.linear.z = 0.3;
        }
    }

    // Angular
    twist.twist.angular.z = -0.75 * msg->rotate;

    cmdVelPub.publish(twist);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control");

    ros::NodeHandle nh;

    ros::Subscriber stateSub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);

    ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, poseCallback);

    ros::Subscriber controlSub = nh.subscribe<exjobb_msgs::Control>("collision_free_control", 10, controlCallback);

    localPosPub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    cmdVelPub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    armingClient = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    setModeClient = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    return 0;
}
