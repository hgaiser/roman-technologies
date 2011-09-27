#include "grijpertest/TemperatureListener.h"

/**
 * Initalize the attributes of TemperatureListener
*/
void TemperatureListener::init()
{
    mTempSensor_sub = mNodeHandle.subscribe("temperatureFeedbackTopic", 10, &TemperatureListener::readTemperatureSensorDataCB, this);
    mStartTime = time(NULL);
    mLastMeasureTime = mStartTime;
    mTemperatureFile.open("/tmp/temperature.txt");
    ROS_INFO("TemperatureListener initialized");
}

/**
 * Called when new sensor data is made available.
*/
void TemperatureListener::readTemperatureSensorDataCB(const grijpertest::TemperaturePtr& msg)
{
    if (time(NULL) - mLastMeasureTime < 15)
        return;
    
    mLastMeasureTime = time(NULL);
    ROS_INFO("Temperatuur is: %d", msg->temperature);
    mTemperatureFile << "time=" << mLastMeasureTime - mStartTime << " temperature=" << msg->temperature << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "temperature");
    TemperatureListener temp_listener;
    temp_listener.init();
    ros::spin();
    return 0;
}
