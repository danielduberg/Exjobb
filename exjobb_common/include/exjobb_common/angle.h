#ifndef EXJOBB_COMMON_Angle
#define EXJOBB_COMMON_Angle

#include <iostream>

namespace exjobb_common
{

class Angle
{
private:
    double angle_;

    double bound(double angle) const;

public:
    Angle(double angle);

    /**
     * @brief Angle::difference
     * @param other
     * @return the difference between the angles (negative if other is to the right)
     */
    double difference(const Angle & other) const;

    /**
     * @brief side
     * @param other
     * @return 1 if other is to the left, -1 if other is to the right, 0 if neither
     */
    int side(const Angle & other) const;

    /**
     * @brief isOnLeftSide
     * @param other
     * @return whether other is to the left or not
     */
    bool isOnLeftSide(const Angle & other) const;

    /**
     * @brief isOnRightSide
     * @param other
     * @return whether other is to the right or not
     */
    bool isOnRightSide(const Angle & other) const;

    double middle(const Angle & other) const;

    double getAngle() const;

    static double getDegrees(double angle);

    static double getRadians(double angle);

    double getDegrees() const;

    double getRadians() const;

    double get() const;

    // http://en.cppreference.com/w/cpp/language/operators
    /*
     * Assignment operator
     *
    */
    // copy assignment
    Angle & operator=(const Angle & other);

    // move assignment
    //Angle & operator=(Angle && other) noexcept;

    /*
     * Increment and decrement
     *
    */
    Angle & operator++();

    Angle operator++(int)
    {
        Angle tmp(*this);   // copy
        operator++();           // pre-increment
        return tmp;             // return old value
    }

    Angle & operator--();

    Angle operator--(int)
    {
        Angle tmp(*this);   // copy
        operator--();           // pre-increment
        return tmp;             // return old value
    }

    /*
     * Binary arithmetic operators
     *
    */
    // compound assignment
    // Add
    Angle & operator+=(const Angle & rhs);

    friend Angle operator+(Angle lhs, const Angle & rhs)
    {
        lhs += rhs; // reuse compound assignment
        return lhs; // return the result by value (uses move constructor)
    }

    /*
    template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
    Angle & operator+=(const T & rhs)
    {
        angle += rhs;
        return *this;
    }

    template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
    friend Angle operator+(Angle lhs, const T & rhs)
    {
        lhs += rhs;
        return lhs;
    }

    template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
    friend T operator+=(T & lhs, const Angle & rhs)
    {
        lhs += rhs.getCorrect();
        return lhs;
    }

    template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
    friend T operator+(T lhs, const Angle & rhs)
    {
        lhs += rhs;
        return lhs;
    }

    template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
    Angle operator=(const T & rhs)
    {
        angle = rhs;
    }
    */

    // compound assignment
    Angle & operator-=(const Angle & rhs);

    friend Angle operator-(Angle lhs, const Angle & rhs)
    {
        lhs -= rhs; // reuse compound assignment
        return lhs; // return the result by value (uses move constructor)
    }

    Angle & operator*=(const Angle & rhs);

    friend Angle operator*(Angle lhs, const Angle & rhs)
    {
        lhs *= rhs;
        return lhs;
    }

    Angle & operator/=(const Angle & rhs);

    friend Angle operator/(Angle lhs, const Angle & rhs)
    {
        lhs /= rhs;
        return lhs;
    }

    /*
     * Relational operators
     *
    */
    friend bool operator<(const Angle & lhs, const Angle & rhs);

    friend bool operator>(const Angle & lhs, const Angle & rhs)
    {
        return rhs < lhs;
    }

    friend bool operator<=(const Angle & lhs, const Angle & rhs)
    {
        return !(lhs > rhs);
    }

    friend bool operator>=(const Angle & lhs, const Angle & rhs)
    {
        return !(lhs < rhs);
    }

    friend bool operator==(const Angle & lhs, const Angle & rhs);

    friend bool operator!=(const Angle & lhs, const Angle & rhs)
    {
        return !(lhs == rhs);
    }

    friend std::ostream& operator<<(std::ostream& out, const Angle & angle);
};

std::ostream& operator<<(std::ostream& out, const Angle & angle)
{
    return out << angle.angle_;
}

}	// namespace exjobbb_common

#endif // EXJOBB_COMMON_Angle
