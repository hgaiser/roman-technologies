#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/JointState.h"
#include <linux/input.h>

#include <roman/Key.h>

#include <iostream>
#include <string>
#include <fcntl.h>

struct input_event ev[64];

/**
 * Handles keys pressed.
*/
int main(int argc, char **argv)
{
    // initialize key listener
    int joystick = open("/dev/ps3bt", O_RDONLY); // try bluetooth first
    if (joystick == -1)
    {
        // bluetooth failed, try usb
        ROS_INFO("BlueTooth device not found, trying USB.");
        joystick = open("/dev/ps3usb", O_RDONLY);
        if (joystick == -1)
        {
            // no ps3 controller found
            ROS_INFO("No Bluetooth and no USB device found ... uh oh.");
            return 0;
        }
    }

    // init ros and publisher
    ros::init(argc, argv, "keyPublisher");
    ros::NodeHandle n;
    ros::Publisher key_pub = n.advertise<roman::Key>("keyTopic", 1);

    ROS_INFO("KeyListener initialized");

    // keep reading key input, until we need to exit
    while (ros::ok())
    {
        int rd = read(joystick, ev, sizeof(struct input_event) * 64);

        // construct Key message with key ids and values
        roman::Key msg;
        for (unsigned int i = 0; i < rd / sizeof(struct input_event); i++)
        {
            msg.keys.push_back(ev[i].code);
            msg.values.push_back(ev[i].value);
        }
        
        // if we added a key, send the message
        if (msg.keys.size())
            key_pub.publish(msg);
        ros::spinOnce();
    }

    close(joystick);

    return 0;
}
