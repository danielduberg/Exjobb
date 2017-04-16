#include <ros/ros.h>

#include <collision_avoidance/basic.h>

Basic::Basic(float radius, float security_distance)
    : radius_(radius)
    , security_distance_(security_distance)
{

}

void Basic::avoidCollision(exjobb_msgs::Control * control, const std::vector<Point> & obstacles, float current_direction, float current_speed)
{
    if (control->go_magnitude == 0)
    {
        // Stay in place
        stayInPlace(control, obstacles);
    }
    else
    {
        // We are moving
        moving(control, obstacles, current_direction, current_speed);
    }
}


void Basic::stayInPlace(exjobb_msgs::Control * control, const std::vector<Point> & obstacles)
{

}


void Basic::moving(exjobb_msgs::Control * control, const std::vector<Point> & obstacles, float current_direction, float current_speed)
{
    float direction_epsilon = 40; // Five degrees
    float speed_epsilon = 0.2; // 0.2 meters per second?

    float direction_diff = std::fabs(control->go_direction - current_direction);
    if (direction_diff > 180)
    {
        direction_diff = 360 - direction_diff;
    }


    if (direction_diff > direction_epsilon &&
            current_speed > speed_epsilon)
    {
        // We are going the wrong way! Stop!
        control->go_direction = current_direction + 180;
        if (control->go_direction > 360)
        {
            control->go_direction -= 360;
        }

        control->go_magnitude = current_speed;

        ROS_ERROR_STREAM("STOPPING!!!");
    }


    float shortest_distance = 10000;
    for (size_t i = 0; i < obstacles.size(); i++)
    {
        if (obstacles[i].x == 0 && obstacles[i].y == 0)
        {
            continue;
        }

        float direction = Point::GetDirectionDegrees(obstacles[i]);



        float distance = Point::getDistance(obstacles[i]) * 0.1;

        if (distance < shortest_distance)
        {
            shortest_distance = distance;
        }
    }

    control->go_magnitude = std::min(control->go_magnitude, shortest_distance);

    // TODO: Take into account the current movement

    // Create a rectangle of the path
}
