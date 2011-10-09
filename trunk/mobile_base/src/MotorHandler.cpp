#include <MotorHandler.h>

/**
 * Called when a Twist message is received over the motor topic.
 */
void MotorHandler::moveCB(const geometry_msgs::Twist& msg)
{
	double left_vel = msg.linear.x - msg.angular.z * WHEEL_RADIUS;
	double right_vel = msg.linear.x + msg.angular.z * WHEEL_RADIUS;

	mRightMotor.setSpeed(right_vel);
	mLeftMotor.setSpeed(left_vel);
}

/**
 * Called when a Twist message is received over the motor topic.
*/
void MotorHandler::tweakCB(const mobile_base::tweak msg)
{
	Motor *motor = mMotorId == MID_LEFT ? &mLeftMotor : &mRightMotor;

	switch (msg.data)
	{
	case PS3_UP:
	case PS3_DOWN:
		motor->mPID[mPIDFocus] += msg.data == PS3_DOWN ? -0.01 : 0.01;
		motor->mPID[mPIDFocus] = std::max(0.0, motor->mPID[mPIDFocus]);
		motor->updatePID();
		motor->printPID();
		break;

	case PS3_LEFT:
	case PS3_RIGHT:
		// toggle through P-I-D
		mPIDFocus = PIDParameter((mPIDFocus + (msg.data == PS3_LEFT ? -1 : 1)) % PID_PARAM_MAX);

		switch (mPIDFocus)
		{
		case PID_PARAM_P: ROS_INFO("P"); break;
		case PID_PARAM_I: ROS_INFO("I"); break;
		case PID_PARAM_D: ROS_INFO("D"); break;
		default: break;
		}

		break;

	case PS3_L1:
	case PS3_R1:
		mMotorId = MotorId(msg.motorID);
		ROS_INFO("SWITCHED TO %s ENGINE", mMotorId == MID_LEFT ? "LEFT" : "RIGHT");
		break;
	}
}

/**
 * Initalize MotorHandler and its attributes.
 */
void MotorHandler::init(char *path)
{
	twist_sub = nh_.subscribe("/movementTopic", 10, &MotorHandler::moveCB, this);
	tweak_sub = nh_.subscribe("/tweakTopic", 10, &MotorHandler::tweakCB, this);
	mLeftMotor.init(path);
	mRightMotor.init(path);

	ROS_INFO("Initializing completed.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MotorHandler");

	char *path=NULL;
	if (argc == 2)
		path = argv[1];

	MotorHandler motorHandler;
	motorHandler.init(path);

	ros::spin();

	return 0;
}

