#include "roman/Controller.h"
#include <sstream>
#include "sensor_msgs/JointState.h"

#define JOINT_COUNT 7

/// names of the joints in the RViz visualization
static const char *jointNames[JOINT_COUNT] =
{
    "base_to_left_finger",
    "left_finger_to_left_finger_top",
    "right_palm_to_right_upper_finger",
    "right_palm_to_right_lower_finger",
    "right_upper_finger_to_right_upper_finger_top",
    "right_lower_finger_to_right_lower_finger_top",
    "base_to_grab_object",
};

/// modifications in yaw rotation for the joints when the gripper is closed
static const float jointYaw[JOINT_COUNT] =
{
    -0.785f,
    -1.04f,
    0.785f,
    0.785f,
    1.04f,
    1.04f,
    -0.375f,
};

/**
 * Sends yaw values for joints. Takes mGripperState into consideration for chosing the yaw value.
*/
void Controller::UpdateJoints()
{
    sensor_msgs::JointState msg;

    // not sure why this is needed, but it won't work without it
    msg.header.stamp = ros::Time::now();

    // JointState message contains vectors with the joint name and the yaw value
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        msg.name.push_back(jointNames[i]);
        msg.position.push_back(mGripperState == GS_CLOSED ? jointYaw[i] : 0.f);
    }

    mJoint_pub.publish(msg);
    ROS_INFO("Changed joints");
}

/**
 * Initalize the attributes of the controller
*/
void Controller::init()
{
    // intialize publishers
    mSensor_pub  = mNodeHandle.advertise<std_msgs::Empty>("sensorTopic", 10);
    mMotor_pub   = mNodeHandle.advertise<roman::MotorControl>("motorTopic", 10);
    mJoint_pub   = mNodeHandle.advertise<sensor_msgs::JointState>("joint_states", 10);

    // initialize subscribers
    mSensor_sub = mNodeHandle.subscribe("sensorFeedbackTopic", 10, &Controller::readSensorDataCB, this);
    mKey_sub    = mNodeHandle.subscribe("keyTopic", 10, &Controller::keyCB, this);

    ROS_INFO("Controller initialized");
}

/**
 * Called when new sensor data is made available.
*/
void Controller::readSensorDataCB(const roman::DistancePtr& msg)
{
    ROS_INFO("I heard: [%d]", msg->distance);

    // default message, open the gripper
    roman::MotorControl mc;
    mc.modeStr = "torque";
    mc.value = 0.02f;
    mc.waitTime = 1000;

    // Is the distance smaller than 10cm and are we not grabbing the object yet? Then close the gripper
    if (msg->distance < 100 && mGripperState != GS_CLOSED)
    {
        mc.value = -mc.value;
        mMotor_pub.publish(mc);
        mGripperState = GS_CLOSED;
    }
    // Is the distance greater than 15cm and are we grabbing the object, then open the gripper.
    else if (msg->distance > 150 && mGripperState != GS_OPEN)
    {
        mMotor_pub.publish(mc);
        mGripperState = GS_OPEN;
    }
}

/**
 * Called when sensor should be activated/deactivated.
*/
void Controller::keyCB(const roman::KeyPtr& msg)
{
    //ROS_INFO("Key Received. Id = %d, value = %d", msg->keys[i], msg->values[i]);

    roman::MotorControl mc;

    // handle complete buffer of keys that have been pressed
    for (size_t i = 0; i < msg->keys.size(); i++)
    {
        // if there is already a key registered as being pushed and its value is now 0,
        // then we no longer have a registered pushed key
        if (mKeyPressed == msg->keys[i] && msg->values[i] == 0)
        {
            mKeyPressed = PS3_NONE;

            // if the key was one of the shoulder buttons, release the motor
            if (msg->keys[i] == PS3_USB_L2 || msg->keys[i] == PS3_USB_R2 ||
                msg->keys[i] == PS3_USB_L1 || msg->keys[i] == PS3_USB_R1 ||
                msg->keys[i] == PS3_BT_L2 || msg->keys[i] == PS3_BT_R2 ||
                msg->keys[i] == PS3_BT_L1 || msg->keys[i] == PS3_BT_R1)
            {
                mc.modeStr = "torque";
                mc.value = 0.f;
                mc.waitTime = 0;
            }
        }
        else if (msg->values[i] > 1)
        {
            // handle key events (create callbacks for this?)
            switch (msg->keys[i])
            {
            case PS3_USB_X:
            case PS3_BT_X:
                if (mKeyPressed != PS3_NONE)
                   break;

                mKeyPressed = PS3Key(msg->keys[i]);
                // open the gripper
                mc.modeStr = "torque";
                mc.value = 0.5f;
                mc.waitTime = 1000;

                ROS_INFO("X BUTTON");
                break;
            case PS3_USB_O:
            case PS3_BT_O:
                if (mKeyPressed != PS3_NONE)
                    break;

                mKeyPressed = PS3Key(msg->keys[i]);
                // close the gripper
                mc.modeStr = "torque";
                mc.value = -0.5f;
                mc.waitTime = 1000;

                ROS_INFO("O BUTTON");
                break;
            case PS3_USB_R2:
            case PS3_USB_L2:
            case PS3_BT_R2:
            case PS3_BT_L2:
            {
                ROS_INFO("SHOULDER BUTTON R2/L2 %d", msg->values[i]);

                mKeyPressed = PS3Key(msg->keys[i]);
                mc.modeStr = "current";
                // interpolate between 0 and 0.5A
                mc.value = 0.5f * (float(msg->values[i]) / 255.f);
                mc.waitTime = 0;

                // close the gripper with L2 shoulder button
                if (msg->keys[i] == PS3_USB_L2 || msg->keys[i] == PS3_BT_L2)
                    mc.value = -mc.value;               
                break;
            }
            case PS3_USB_R1:
            case PS3_USB_L1:
            case PS3_BT_R1:
            case PS3_BT_L1:
            {
                ROS_INFO("SHOULDER BUTTON R1/L1 %d", msg->values[i]);

                mKeyPressed = PS3Key(msg->keys[i]);
                mc.modeStr = "torque";
                // interpolate between 0 and 0.05Nm
                mc.value = 0.05f * (float(msg->values[i]) / 255.f);
                mc.waitTime = 0;

                // close the gripper with L1 shoulder button
                if (msg->keys[i] == PS3_USB_L1 || msg->keys[i] == PS3_BT_L1)
                    mc.value = -mc.value;               
                break;
            }
            case PS3_USB_T:
            case PS3_BT_T:
            {
                if (mKeyPressed != PS3_NONE)
                    break;

                // toggle the sensor
                mKeyPressed = PS3Key(msg->keys[i]);
                std_msgs::Empty msg;
                mSensor_pub.publish(msg);
                ROS_INFO("SENSOR TOGGLED");
                break;
            }
            default:
                break;
            }
        }
    }

    // is our message filled?
    if (mc.modeStr.empty() == false)
    {
        if (mGripperState != GS_OPEN && mc.value > 0.f)
        {
            // we are opening the gripper
            mGripperState = GS_OPEN;
            UpdateJoints();
        }
        else if (mGripperState != GS_CLOSED && mc.value < 0.f)
        {
            // we are closing the gripper
            mGripperState = GS_CLOSED;
            UpdateJoints();
        }

        // print the message for debugging
        std::stringstream ss("");
        ss << mc.modeStr << " " << mc.value << " " << mc.waitTime;
        ROS_INFO("message: %s", ss.str().c_str());

        // publish the message to the motor handler
        mMotor_pub.publish(mc);
    }
}

int main(int argc, char **argv)
{
    // init ros and controller
    ros::init(argc, argv, "controller");
    Controller controller;
    controller.init();
    ros::spin();
    return 0;
}
