#include <exjobb_common/angle.h>

namespace exjobb_common
{

# define M_PI           3.14159265358979323846  /* pi */

Angle::Angle(double angle = 0)
    : angle_(angle)
{

}

double Angle::bound(double angle) const
{
    while (angle < 0)
    {
        angle += 360;
    }

    while (angle >= 360)
    {
        angle -= 360;
    }

    return angle;
}

double Angle::getDegrees(double angle)
{
    // TODO: Should this make sure it is between 0-360?
    return angle * 180.0 / M_PI;
}

double Angle::getRadians(double angle)
{
    // TODO: Should this make sure it is between 0-2*pi?
    return angle * M_PI / 180.0;
}

/**
 * @brief Angle::difference
 * @param other
 * @return the difference between the angles (negative if other is to the right)
 */
double Angle::difference(const Angle & other) const
{
    // TODO: should this only return positive value?
    // If this is changed then change side too

    double diff = other.angle_ - angle_;

    if (diff > 180)
    {
        diff -= 360;
    }
    else if (diff < -180)
    {
        diff += 360;
    }

    return diff;
}

int Angle::side(const Angle & other) const
{
    double diff = difference(other);

    if (diff < 0)
    {
        return -1;  // other is on the right side
    }
    else if (diff > 0)
    {
        return 1;   // other is on the left side
    }

    return 0;       // other is on neither side
}

bool Angle::isOnLeftSide(const Angle & other) const
{
    return (side(other) == 1);
}

bool Angle::isOnRightSide(const Angle & other) const
{
    return (side(other) == -1);
}

double Angle::middle(const Angle & other) const
{
    double mid_diff = difference(other) / 2.0;

    double mid = angle_ + mid_diff;

    return bound(mid);
}

double Angle::getDegrees() const
{
    return angle_;
}

double Angle::getRadians() const
{
    return getRadians(angle_);
}

double Angle::get() const
{
    return angle_;
}

} // namespace exjobb_common
