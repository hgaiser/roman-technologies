#ifndef __ULTRASONE_H
#define __ULTRASONE_H

#include <ros/ros.h>
#include "grijpertest/Temperature.h"
#include <fstream>
#include <time.h>

/// Reads temperature sensor and stops the motor when overheated
class TemperatureListener
{
protected:
    ros::NodeHandle mNodeHandle;   // ROS node handle

    ros::Subscriber mTempSensor_sub;

    time_t mStartTime;
    time_t mLastMeasureTime;
    std::ofstream mTemperatureFile;
    int mTemperatures[30];
    int mTemperatureIndex;
public:
    /// Constructor
    TemperatureListener() : mNodeHandle(""){ }

    /// Destructor
    ~TemperatureListener()
    {
        mTemperatureFile.close();
        mNodeHandle.shutdown();
    }

    /// Initialize node
    void init();

    void readTemperatureSensorDataCB(const grijpertest::TemperaturePtr& msg);
};

#endif /* __CONTROLLER_H */
