#include <BaseController.h>

/**
 * Reads the speed feedback from MotorHandler
 */
void BaseController::readCurrentSpeed(const geometry_msgs::Twist& msg)
{
	mCurrentSpeed = msg;
}

/**
 * Initialise the attributes of the controller
 */
void BaseController::init()
{
	//initialise subscribers
	mKey_sub = mNodeHandle.subscribe("joy", 1, &BaseController::keyCB, this);
	mSpeed_sub = mNodeHandle.subscribe("speedFeedbackTopic", 1, &BaseController::readCurrentSpeed, this);
	mTwist_pub = mNodeHandle.advertise<geometry_msgs::Twist>("movementTopic", 1);
	mTweak_pub = mNodeHandle.advertise<mobile_base::tweak>("tweakTopic", 10);

	ROS_INFO("BaseController initialised");
}

/**
 * Handles key events from the PS3 controller.
 */
void BaseController::keyCB(const sensor_msgs::Joy& msg){

	geometry_msgs::Twist twist_msg;
	mobile_base::tweak button;

	//No button is pressed, so sum of vector is zero -> stand still
	if (std::accumulate(msg.buttons.begin(), msg.buttons.end(), 0) == 0)
	{

		//Nothing is pressed and analog stick is not tilted
		if (msg.axes[PS3_LEFT_HORIZONTAL] == 0)
		{
			twist_msg.linear.x = 0;
			twist_msg.angular.z = 0;
			mTwist_pub.publish(twist_msg);

			mKeyPressed = PS3_NONE;
		}

		//Turn at current position
		else
		{
			twist_msg.linear.x = 0;
			twist_msg.angular.z = 0.5f * float(msg.axes[PS3_LEFT_HORIZONTAL]);

			mTwist_pub.publish(twist_msg);
			mKeyPressed = PS3_NONE;
		}
	}
	else
	{
		twist_msg.linear.x = 0;
		twist_msg.angular.z = 0;
		for (size_t i = 0; i < msg.buttons.size(); i++)
		{
			if(msg.buttons[i] == 0)
				continue;

			switch(i)
			{
			//Accelerate when X button is pressed
			case PS3_X:
				twist_msg.linear.x = -MAX_LINEAR_SPEED * float(msg.axes[PS3_X]);
				if (msg.axes[PS3_LEFT_HORIZONTAL])
				{
					//Depending on the amplitude of the joystick
					twist_msg.angular.z = msg.axes[PS3_LEFT_HORIZONTAL] > 0 ? calcRobotAngularSpeed() : -calcRobotAngularSpeed();

					if (abs(calcRobotAngularSpeed()) > MAX_ANGULAR_SPEED)
						twist_msg.angular.z = msg.axes[PS3_LEFT_HORIZONTAL] > 0 ? MAX_ANGULAR_SPEED : -MAX_ANGULAR_SPEED;
				}

				mTwist_pub.publish(twist_msg);
				break;

				//Go in reverse when Square button is pressed
			case PS3_S:
				twist_msg.linear.x = MAX_LINEAR_SPEED * float(msg.axes[PS3_S]);
				if (msg.axes[PS3_LEFT_HORIZONTAL])
				{
					twist_msg.angular.z = msg.axes[PS3_LEFT_HORIZONTAL] > 0 ? calcRobotAngularSpeed() : -calcRobotAngularSpeed();

					if (calcRobotAngularSpeed() > MAX_ANGULAR_SPEED)
						twist_msg.angular.z = msg.axes[PS3_LEFT_HORIZONTAL] > 0 ? MAX_ANGULAR_SPEED : -MAX_ANGULAR_SPEED;
				}

				mTwist_pub.publish(twist_msg);
				break;


				//Brake if O button has been pressed
			case PS3_O:
				twist_msg.linear.x = 0;
				twist_msg.angular.z = 0;
				mTwist_pub.publish(twist_msg);
				break;

				//Tweak buttons for PID control
			case PS3_UP:
				if(mKeyPressed != PS3_NONE)
					break;

				button.data = PS3_UP;
				mTweak_pub.publish(button);
				mKeyPressed = PS3_UP;
				break;

			case PS3_RIGHT:
				if(mKeyPressed != PS3_NONE)
					break;

				button.data = PS3_RIGHT;
				mTweak_pub.publish(button);
				mKeyPressed = PS3_RIGHT;
				break;

			case PS3_DOWN:
				if(mKeyPressed != PS3_NONE)
					break;

				button.data = PS3_DOWN;
				mTweak_pub.publish(button);
				mKeyPressed = PS3_DOWN;
				break;

			case PS3_LEFT:
				if(mKeyPressed != PS3_NONE)
					break;

				button.data = PS3_LEFT;
				mTweak_pub.publish(button);
				mKeyPressed = PS3_LEFT;
				break;

			default:
				break;
			}
		}
	}
}

int main(int argc, char **argv)
{
	// init ros and controller
	ros::init(argc, argv, "controller");
	BaseController base_controller;

	char *path = NULL;
	if (argc == 2)
		path = argv[1];

	base_controller.init();
	ros::spin();

	return 0;
}
