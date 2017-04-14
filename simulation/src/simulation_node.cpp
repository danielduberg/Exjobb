#include <ros/ros.h>

#include <gazebo_msgs/ContactsState.h>

void contactCallback(const gazebo_msgs::ContactsState::ConstPtr & msg)
{
    if (msg->states.size() != 0)
    {
        // Print at most once every 5 seconds
        ROS_ERROR_STREAM_THROTTLE(5, msg->header.frame_id << " - made contact");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulation");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/contact_sensor_state", 10, contactCallback);

    ros::spin();

    return 0;
}
