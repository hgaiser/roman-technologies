#include "roman/mobile_manipulator/BaseController.h"

char *path=NULL;

/**
 * Initalize the attributes of the controller
*/
void BaseController::init()
{
    // intialize subscribers
    mKey_sub    = mNodeHandle.subscribe("keyTopic", 10, &BaseController::keyCB, this);

    leftEngine  = new Motor(106);
    rightEngine = new Motor(107);
    armEngine   = new Motor(108);

    leftEngine->init(path);
    rightEngine->init(path);
    armEngine->init(path);

    ROS_INFO("BaseController initialized");
}

/**
 * Drives the base forward
*/
  void BaseController::moveForward(double speed){
      leftEngine->setSpeed(speed);
      rightEngine->setSpeed(speed);
  }

  /**
   * Drives the base backward
  */
  void BaseController::moveBackward(double speed){
      leftEngine->setSpeed(-speed);
      rightEngine->setSpeed(-speed);
  }

  /**
   * Turns the base to the left
  */
  void BaseController::turnLeft(double speed){
      leftEngine->setSpeed(-speed);
      rightEngine->setSpeed(speed);
  }

  /**
   * Turns the base to the right
  */
  void BaseController::turnRight(double speed){
      leftEngine->setSpeed(speed);
      rightEngine->setSpeed(-speed);
  }

  /**
   * Stops the base
  */
  void BaseController::brake(){
      leftEngine->setSpeed(0);
  }

  //TODO Design and implement movement interface with ps3 controller
/**
 * Called when sensor should be activated/deactivated.
*/
void BaseController::keyCB(const roman::KeyPtr& msg)
{
    for (size_t i = 0; i < msg->keys.size(); i++)
    {
        // if there is already a key registered as being pushed and its value is now 0,
        // then we no longer have a registered pushed key
        if (mKeyPressed == msg->keys[i] && msg->values[i] == 0)
        {
            mKeyPressed = PS3_NONE;

            if (msg->values[i] > 1)
            {
                // handle key events (create callbacks for this?)
                switch (msg->keys[i])
                {
                case PS3_USB_X:
                case PS3_BT_X:
                    if (mKeyPressed != PS3_NONE)
                        break;
                    ROS_INFO("X BUTTON");
                    break;
                case PS3_USB_O:
                case PS3_BT_O:
                    if (mKeyPressed != PS3_NONE)
                        break;
                    ROS_INFO("O BUTTON");
                    break;
                case PS3_USB_R2:
                case PS3_USB_L2:
                case PS3_BT_R2:
                case PS3_BT_L2:
                {
                    ROS_INFO("SHOULDER BUTTON R2/L2 %d", msg->values[i]);

                    break;
                }
                case PS3_USB_R1:
                case PS3_USB_L1:
                case PS3_BT_R1:
                case PS3_BT_L1:
                {
                    ROS_INFO("SHOULDER BUTTON R1/L1 %d", msg->values[i]);
                    break;
                }
                case PS3_USB_T:
                case PS3_BT_T:
                {
                    if (mKeyPressed != PS3_NONE)
                        break;
                    ROS_INFO("SENSOR TOGGLED");
                    break;
                }
                default:
                    break;
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    // init ros and controller
    ros::init(argc, argv, "controller");
    BaseController base_controller;

    if (argc == 2)
        path = argv[1];

    base_controller.init();
    ros::spin();

    return 0;
}
