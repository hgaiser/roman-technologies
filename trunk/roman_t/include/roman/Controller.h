#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <ros/ros.h>
#include <roman/Distance.h>
#include <CDxlGeneric.h>
#include <roman/MotorControl.h>
#include <roman/Key.h>
#include "std_msgs/Empty.h"

enum GripperState
{
    GS_NONE = -1,
    GS_OPEN,
    GS_CLOSED,
};

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
    ros::NodeHandle mNodeHandle;   // ROS node handle

    ros::Publisher mSensor_pub;
    ros::Publisher mMotor_pub;
    ros::Publisher mJoint_pub;

    ros::Subscriber mSensor_sub;
    ros::Subscriber mKey_sub;

    GripperState mGripperState;
    PS3Key mKeyPressed;

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
