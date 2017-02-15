#include "ros/ros.h"

// Own
#include "exjobb_msgs/Distance.h"

class Sensor {
public:
    float offset, angle;

    std::string topic;
    
private:
	std::vector<float> distances;
	
	ros::Time lastUpdate;


public:
    Sensor(float offset, float angle, std::string topic);
    
    // Copy constructor
    Sensor(const Sensor & other);
    
    // Copy constructor
    Sensor(Sensor & other);
    
    void callback(const exjobb_msgs::Distance::ConstPtr & msg);
    
    /**
     * @brief timeSinceLastUpdate
     * @return Time in seconds since last update
     */
    float timeSinceLastUpdate();
    
    /**
     * @brief getDistances
     * @return Distances
     */
    const std::vector<float> & getDistances();
};

