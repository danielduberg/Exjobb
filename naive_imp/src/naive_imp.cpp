#include "ros/ros.h"

// C++ general
#include "vector"

// Msgs
#include "sensor_msgs/Image.h"

// Own
#include "naive_imp/sensor.h"

#include "exjobb_msgs/Distance.h"

// Resolution
// Distance

int resolution = 90;
float closestDistance = -1;

float maxDelayForSensor = 1.0;   // 1 second

float frequency = 10;

std::list<Sensor *> sensors;

bool isSensorRecentlyUpdated(Sensor & sensor) {
    float timeSinceUpdate = sensor.timeSinceLastUpdate();
    std::cout << "Time: " << timeSinceUpdate << std::endl;
    if (timeSinceUpdate == -1) {
        return false;
    }

    return  timeSinceUpdate <= maxDelayForSensor;
}

void combineDistances(std::vector<float> & distances) {
    std::cout << "GO" << std::endl;
    for (std::list<Sensor *>::iterator sensor = sensors.begin(); sensor != sensors.end(); sensor++) {
        std::cout << "1" << std::endl;
        // Check that the sensor has been updated recently enough.
        // If it has taken too long for it to update we regard it as not working
        if (!isSensorRecentlyUpdated(**sensor)) {
            // The sensor has not been updated for a while so we think it might be dead...
            continue;
        }

        std::cout << "2" << std::endl;
        int numEntriesForSensor = round((resolution / 360.0f) * (*sensor)->angle);
        if (numEntriesForSensor <= 1) {
            std::cout << resolution << ", " << (*sensor)->angle << std::endl;
            continue;
        }

        std::cout << "3" << std::endl;
        float sensorDistances[numEntriesForSensor];

        std::cout << "4" << std::endl;
        for (size_t j = 0; j < numEntriesForSensor; j++) {
            std::cout << "4.1" << std::endl;
            // TODO: Check if correct
            int correspondingIndex = round(((*sensor)->getDistances().size() - 1) * (j / ((float) numEntriesForSensor - 1)));

            std::cout << "4.2" << std::endl;
            std::cout << (*sensor)->topic << std::endl;
            std::cout << correspondingIndex << std::endl;
            std::cout << (*sensor)->getDistances().size() << std::endl;
            std::cout << numEntriesForSensor << std::endl;
            sensorDistances[j] = (*sensor)->getDistances()[correspondingIndex];

            std::cout << "4.3" << std::endl;
            // Take into account the values inbetween
            if (j > 0) {
                int prevCorrespondingIndex = round(((*sensor)->getDistances().size() - 1) * ((j-1) / ((float) numEntriesForSensor - 1)));

                int halfIndexDiff = (correspondingIndex - prevCorrespondingIndex) / 2;

                for (size_t k = correspondingIndex - halfIndexDiff; k < correspondingIndex; k++) {
                    // Update if closer
                    if ((*sensor)->getDistances()[k] < sensorDistances[j]) {
                        sensorDistances[j] = (*sensor)->getDistances()[k];
                    }
                }
            }

            std::cout << "4.4" << std::endl;
            if (j + 1 < numEntriesForSensor) {
                int nextCorrespondingIndex = round(((*sensor)->getDistances().size() - 1) * ((j+1) / ((float) numEntriesForSensor - 1)));

                int halfIndexDiff = (nextCorrespondingIndex - correspondingIndex) / 2;

                for (size_t k = correspondingIndex; k < correspondingIndex + halfIndexDiff; k++) {
                    // Update if closer
                    if ((*sensor)->getDistances()[k] < sensorDistances[j]) {
                        sensorDistances[j] = (*sensor)->getDistances()[k];
                    }
                }
            }
        }

        std::cout << "5" << std::endl;
        for (size_t j = 0; j < numEntriesForSensor; j++) {
            std::cout << resolution << ", " << (*sensor)->offset << ", " << j << " = " << ((int) round((resolution / 360.0f) * (*sensor)->offset) + j) % resolution << std::endl;
            distances[((int) round((resolution / 360.0f) * (*sensor)->offset) + j) % resolution] = sensorDistances[j];
        }
    }

    std::cout << "END" << std::endl;
}

// Publishes the distances from all the sensors combined
void publishDistances(ros::Publisher & pub) {
    // Init distances
    std::vector<float> distances(resolution, closestDistance);

    std::cout << "Start - Combine" << std::endl;
    combineDistances(distances);


    // Publish the combined distances!
    exjobb_msgs::Distance msg;
    // Add header
    msg.header.stamp = ros::Time::now();
    msg.distances = distances;

    pub.publish(msg);
    std::cout << "End - Combine" << std::endl;
}

void initSubscribers(ros::NodeHandle & nh, std::list<ros::Subscriber> & subs) {
    std::vector<float> offsets;
    std::vector<float> angles;
    std::vector<std::string> topics;

    nh.getParam("/offsets", offsets);
    nh.getParam("/angles", angles);
    nh.getParam("/topics", topics);

    for (size_t i = 0; i < topics.size(); i++) {
        Sensor * temp = new Sensor(offsets[i], angles[i], topics[i]);
        sensors.push_back(temp);
        subs.push_back(nh.subscribe(topics[i], 10, &Sensor::callback, sensors.back()));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "naive_imp");

    ros::NodeHandle nh;

    std::list<ros::Subscriber> subs;

    initSubscribers(nh, subs);


    ros::Publisher pub = nh.advertise<exjobb_msgs::Distance>("processed/distances", 1000);

    ros::Rate loop_rate(frequency);

    while (ros::ok()) {
        std::cout << "Getting shit done!" << std::endl;
        ros::spinOnce();
        std::cout << "Getting shit done 2!" << std::endl;

        publishDistances(pub);

        std::cout << "Enter sleep" << std::endl;
        loop_rate.sleep();
        std::cout << "Finish sleep" << std::endl;
    }

    return 0;
}
