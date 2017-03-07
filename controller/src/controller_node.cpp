#include "ros/ros.h"

#include "sensor_msgs/Joy.h"

#include "controller/Control.h"

#define PI 3.14159265

ros::Publisher pub;

float getDirection(float x, float y) {
    if (x == 0 && y == 0) {
        return 90;
    }
    return std::fmod(((std::atan2(x, y) * 180 / PI) + 360), 360);
}

float getDistanceFromOrigo(float x, float y) {
    return std::sqrt((x * x) + (y * y));
}

void controllerCallback(const sensor_msgs::Joy::ConstPtr & msg) {
    // Axes
    float ls_leftRight = msg->axes[0]; // Left stick, Left-Right (Left = +, Right = -)

    float ls_upDown = msg->axes[1]; // Left stick, Up-Down (Up = +, Down = -)

    float rs_leftRight = msg->axes[2]; // Right stick, Left-Right (Left = +, Right = -)

    float rs_upDown = msg->axes[5]; // Left stick, Up-Down (Up = +, Down = -)

    float l2 = msg->axes[3]; // Not pressed = 1, fully pressed = -1

    float r2 = msg->axes[4]; // Not pressed = 1, fully pressed = -1

    float dpad_leftRight = msg->axes[6]; // Left = 1, Right -1, None = 0

    float dpad_upDown = msg->axes[7]; // Up = 1, Down -1, None = 0

    // Buttons
    // Pressed = 1, not pressed = 0
    int square_button = msg->buttons[0];

    int x_button = msg->buttons[1];

    int circle_button = msg->buttons[2];

    int triangle_button = msg->buttons[3];

    int l1_button = msg->buttons[4];

    int r1_button = msg->buttons[5];

    int l2_button = msg->buttons[6];

    int r2_button = msg->buttons[7];

    int share_button = msg->buttons[8];

    int start_button = msg->buttons[9];

    int ls_button = msg->buttons[10];

    int rs_button = msg->buttons[11];

    int ps_button = msg->buttons[12];

    int touchpad_button = msg->buttons[13];


    controller::Control control;

    control.goDirection = getDirection(ls_upDown, -ls_leftRight);
    control.goMagnitude = getDistanceFromOrigo(ls_upDown, ls_leftRight);
    control.lookDirection = getDirection(rs_upDown, -rs_leftRight);
    control.rotate = (std::fabs(r2 - 1.0) + (l2 - 1.0)) / 2.0;
    control.lift = (x_button == 1) ? true : false;
    control.land = (square_button == 1) ? true : false;

    control.header.stamp = ros::Time::now();
    control.header.frame_id = "Controller";

    pub.publish(control);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");

    ros::NodeHandle nh;

    pub = nh.advertise<controller::Control>("control", 1000);

    ros::Subscriber sub = nh.subscribe("joy", 1000, controllerCallback);

    ros::spin();

    return 0;
}
