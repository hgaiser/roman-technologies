/*
 * AutonomeController.h
 *
 *  Created on: Oct 20, 2011
 *      Author: hans
 */

#ifndef AUTONOMECONTROLLER_H_
#define AUTONOMECONTROLLER_H_

#include <mobile_base/sensorFeedback.h>
#include <mobile_base/BaseMotorControl.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include "BaseController.h"

#define EMERGENCY_STOP_DISTANCE 60

enum BumperState
{
	BUMPER_UNKNOWN = 0,
			BUMPER_FRONT,
			BUMPER_REAR,
			BUMPER_REAR_LEFT,
			BUMPER_REAR_RIGHT
};

/// Controller for autonomous control.
class AutonomeController
{
private:
	ros::NodeHandle mNodeHandle;    		/// ROS node handle
	ros::Subscriber mSensorFeedback_sub; 	/// Subscriber to arduino Ultrasone sensor feedback
	ros::Subscriber mBumperFeedback_sub;	/// Subcriber to arduino Bumper sensor feedback
	ros::Publisher	mDisableMotor_pub;		/// Publisher for disabling forward/backward movement
	ros::Publisher  mMovement_pub;			/// Publisher to control the motors

public:
	AutonomeController() {};

	void init();
	void ultrasoneFeedbackCB(const mobile_base::sensorFeedback &msg);
	void bumperFeedbackCB(const std_msgs::UInt8 &msg);
};

#endif /* AUTONOMECONTROLLER_H_ */
