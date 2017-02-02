#include "ros/ros.h"

#include "std_msgs/String.h"

class Sensor {
public:
    float offset, angle;

    std::string topic;
    
private:
	std::vector<float> distances;
	
	ros::Time lastUpdate;


public:
    Sensor(float offset, float angle, std::string topic);
    
    void callback(const std_msgs::String::ConstPtr & msg);
    
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

