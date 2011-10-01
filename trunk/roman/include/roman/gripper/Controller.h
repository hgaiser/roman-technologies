#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <ros/ros.h>
#include <roman/gripper/Distance.h>
#include <CDxlGeneric.h>
#include <roman/gripper/MotorControl.h>
#include <roman/gripper/Key.h>
#include "std_msgs/Empty.h"

/// Different gripper states.
enum GripperState
{
    GS_NONE = -1,
    GS_OPEN,
    GS_CLOSED,
};

/// Key ids for bluetooth connected PS3 controller (PS3Joy) and USB connected PS3 controller
enum PS3Key
{
    PS3_NONE = 0,
	PS3_BT_L2 = 8,
	PS3_BT_R2 = 9,
	PS3_BT_L1 = 10,
	PS3_BT_R1 = 11,
	PS3_BT_T = 12,
	PS3_BT_O = 13,
	PS3_BT_X = 14,

	PS3_USB_L2 = 48,
	PS3_USB_R2 = 49,
	PS3_USB_L1 = 50,
	PS3_USB_R1 = 51,
	PS3_USB_T = 52,
	PS3_USB_O = 53,
	PS3_USB_X = 54,
};

/// Receives sensor feedback and sends gripper control messages.
class Controller
{
protected:
    ros::NodeHandle mNodeHandle;    /// ROS node handle

    ros::Publisher mSensor_pub;     /// Sensor topic, used for toggling the sensor
    ros::Publisher mMotor_pub;      /// Motor topic, used for controlling the motor
    ros::Publisher mJoint_pub;      /// Joint topic, used for changing the joints in RViz

    ros::Subscriber mSensor_sub;    /// Sensor feedback topic, provides sensor data
    ros::Subscriber mKey_sub;       /// Key input topic, provides key input data (id, value)

    GripperState mGripperState;     /// Defines the state of the gripper
    PS3Key mKeyPressed;             /// Contains the current key being pressed (PS3_NONE for no key)

public:
    /// Constructor
    Controller() : mNodeHandle(""), mGripperState(GS_NONE), mKeyPressed(PS3_NONE) { }
    
    /// Destructor
    ~Controller()
    {
        mNodeHandle.shutdown();
    }

    /// Updates joints in RViz
    void UpdateJoints();

    /// Initialize node
    void init();

    void keyCB(const roman::KeyPtr& msg);
    void readSensorDataCB(const roman::DistancePtr& msg);
};    

#endif /* __CONTROLLER_H */
