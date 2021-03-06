#include <ros/ros.h>

#include <sensor_msgs/Joy.h>

#include <exjobb_msgs/Control.h>

#define PI 3.14159265

ros::Publisher pub;

float look_direction = 0;

bool turn_to_where_going = false;   // If we should turn so we always face where we are going
int triangle_button_previous_state = 0;
int option_button_previous_state = 0;

float getDirection(float x, float y)
{
    if (x == 0 && y == 0)
    {
        return 0;
    }

    // 630 = 360 + 360 - 90
    return std::fmod(round(((std::atan2(x, y) * 180.0 / PI) + 630.0)), 360.0);
}

float getDistanceFromOrigo(float x, float y)
{
    return std::sqrt((x * x) + (y * y));
}

void controllerCallback(const sensor_msgs::Joy::ConstPtr & msg)
{
    // Axes
    float ls_left_right = msg->axes[0]; // Left stick, Left-Right (Left = +, Right = -)

    float ls_up_down = msg->axes[1]; // Left stick, Up-Down (Up = +, Down = -)

    float rs_left_right = msg->axes[2]; // Right stick, Left-Right (Left = +, Right = -)

    float rs_up_down = msg->axes[5]; // Left stick, Up-Down (Up = +, Down = -)

    float l2 = msg->axes[3]; // Not pressed = 1, fully pressed = -1

    float r2 = msg->axes[4]; // Not pressed = 1, fully pressed = -1

    int dpad_left_right = msg->axes[6]; // Left = 1, Right -1, None = 0

    int dpad_up_down = msg->axes[7]; // Up = 1, Down -1, None = 0

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

    int option_button = msg->buttons[9];

    int ls_button = msg->buttons[10];

    int rs_button = msg->buttons[11];

    int ps_button = msg->buttons[12];

    int touchpad_button = msg->buttons[13];

    switch (dpad_left_right)
    {
    case 1:
        look_direction = 90;    // Look to the left
        break;
    case -1:
        look_direction = 270;   // Look to the right
        break;
    }

    switch (dpad_up_down)
    {
    case 1:
        look_direction = 0;     // Look forward
        break;
    case -1:
        look_direction = 180;   // Look backwards
        break;
    }

    // Only change state when button has been pressed
    if (triangle_button != triangle_button_previous_state)
    {
        triangle_button_previous_state = triangle_button;

        if (triangle_button == 1)
        {
            turn_to_where_going = !turn_to_where_going;
        }
    }

    // Only arm/disarm when button has been pressed
    /*
    if (option_button != option_button_previous_state)
    {
        option_button_previous_state = option_button;
    }
    else
    {
        option_button = 0;  // So we can hold the button down for a while without it having an affect
    }
    */

    exjobb_msgs::Control control;

    control.go_direction = getDirection(ls_up_down, -ls_left_right);
    control.go_magnitude = std::min(getDistanceFromOrigo(ls_up_down, ls_left_right), 1.0f);

    if (getDistanceFromOrigo(rs_up_down, rs_left_right) > 0.1)
    {
        look_direction = getDirection(rs_up_down, -rs_left_right);
    }
    else if (getDistanceFromOrigo(ls_up_down, ls_left_right) > 0.1)
    {
        look_direction = getDirection(ls_up_down, -ls_left_right);
    }

    if (getDistanceFromOrigo(rs_up_down, rs_left_right) == 0)
    {
        control.look_direction = look_direction;
    }
    else
    {
        control.look_direction = getDirection(rs_up_down, -rs_left_right);
    }

    control.rotate = (std::fabs(l2 - 1.0) + (r2 - 1.0)) / 2.0;
    control.lift = (x_button == 1) ? true : false;
    control.land = (square_button == 1) ? true : false;
    control.turn_to_where_going = turn_to_where_going;
    control.arm = (option_button == 1) ? true : false;
    control.disarm = (share_button == 1) ? true : false;

    control.header.stamp = ros::Time::now();
    control.header.frame_id = "Controller";

    pub.publish(control);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    ros::NodeHandle nh;

    pub = nh.advertise<exjobb_msgs::Control>("control", 1000);

    ros::Subscriber sub = nh.subscribe("joy", 1000, controllerCallback);

    ros::spin();

    return 0;
}
