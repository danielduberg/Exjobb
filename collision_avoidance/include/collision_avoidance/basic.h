#ifndef BASIC_H
#define BASIC_H

#include <exjobb_msgs/Control.h>
#include <collision_avoidance/point.h>

class Basic
{
private:
    float radius_, security_distance_;

    void stayInPlace(exjobb_msgs::Control * control, const std::vector<Point> & obstacles, float current_direction, float current_speed);

    void moving(exjobb_msgs::Control * control, const std::vector<Point> & obstacles, float current_direction, float current_speed);

public:
    Basic(float radius, float security_distance);

    void avoidCollision(exjobb_msgs::Control * control, const std::vector<Point> & obstacles, float current_direction, float current_speed);
};

#endif // BASIC_H
