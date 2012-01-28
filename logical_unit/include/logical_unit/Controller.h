/*
 * Controller.h
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <audio_processing/speech.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "yaml-cpp/yaml.h"
#include "fstream"
#include <map>
#include "image_processing/FindObject.h"
#include "head/PitchYaw.h"
#include "arm/armJointPos.h"
#include "head/Emotion.h"
#include "image_processing/SetActive.h"

#define MIN_ARM_Z_VALUE (-0.321)
#define MAX_ARM_Z_VALUE (0.1725)
#define MAX_ARM_X_VALUE (0.20)
#define MIN_ARM_X_VALUE (-0.3595)

#define HEAD_INIT_X  0.0
#define HEAD_INIT_Z  0.0

#define HEAD_SLEEP_X  0.0
#define HEAD_SLEEP_Z  -0.6

#define VIEW_OBJECTS_ANGLE	0.4
#define FOCUS_FACE_ANGLE (-0.2)

#define TARGET_YAW_THRESHOLD 0.05
#define TARGET_DISTANCE_THRESHOLD 0.1
#define TARGET_DISTANCE 0.95
// #define GRAB_TARGET_DISTANCE 0.4
#define GRAB_TARGET_SPEED 0.1
#define GRAB_TARGET_TIME 5.0
#define CLEAR_TABLE_DISTANCE (-0.4)
#define LIFT_OBJECT_DISTANCE 0.05

#define LOCK_STARTUP_TIME	 0.5
#define FIND_OBJECT_DURATION 5.0

#define COLA_ID 18904
#define JUICE_ID 18906

#define NEUTRAL_AROUSAL	1

#define HEAD_FREE_THRESHOLD 0.001
#define BASE_FREE_THRESHOLD 0.001
#define ARM_FREE_THRESHOLD 0.001

#define GRAB_OBJECT_Z_OFFSET 0.05

#define MAX_GRAB_ATTEMPTS 3

enum commandValue
{
	NOTHING = -1,
	WAKE_UP,
	EVA,
	BEER,
	JUICE,
	COKE,
	COLA,
	RELEASE,
	SLEEP,
};

enum Emotions
{
	NEUTRAL = 0,
	HAPPY,
	SAD,
	SURPRISED,
};

enum Locks
{
	LOCK_NONE = 0,
	LOCK_PATH,
	LOCK_BASE,
	LOCK_HEAD,
	LOCK_ARM,
	LOCK_GRIPPER,
};

enum GripperState
{
    GS_NONE = -1,
    GS_OPEN,
    GS_CLOSED,
};

class Controller
{
private:
	ros::NodeHandle mNodeHandle;

	ros::Subscriber mSpeechSubscriber;				/// Listens to speech processed by SpeechRecognition node
	ros::Subscriber mBaseGoalSubscriber;			/// Listens to path distances to see whether goal is reached
	ros::Subscriber mNavigationStateSubscriber;		///	Listens to the state of the navigation algorithm
	ros::Subscriber mHeadSpeedSubscriber;			/// Listens to head motor speeds
	ros::Subscriber mBaseSpeedSubscriber;			/// Listens to base motor speeds
	ros::Subscriber mArmSpeedSubscriber;			/// Listens to arm motor speeds
	ros::Subscriber mGripperStateSubscriber;		/// Listens to gripper state

	ros::Publisher mGripperCommandPublisher;		/// Publishes commands to gripper Ultrasone controller
	ros::Publisher mBaseGoalPublisher;				/// Publishes goal commands to PathPlanner
	ros::Publisher mArmPositionPublisher;			/// Publishes coordinates to AutonomeArmController
	ros::Publisher mHeadPositionPublisher;			/// Publishes coordinates to AutonomeHeadController
	ros::Publisher mRotateBasePublisher;			/// Publishes rotation angle for base
	ros::Publisher mPositionBasePublisher;			/// Publishes position for base
	ros::Publisher mEmotionPublisher;				/// Publishes emotions for the head
	ros::Publisher mBaseSpeedPublisher;				/// Publishes twist messages for the mobile base

	ros::ServiceClient mFindObjectClient;			/// Service client for finding the object
	ros::ServiceClient mSetFaceFocusClient;			/// Service client for activating face detection

	tf::TransformListener mTransformListener;		/// Fills mOriginalPosition
	geometry_msgs::Pose mOriginalPosition;			/// Keeps track of the original position of the robot in the map
	geometry_msgs::Pose mGoal;						/// Stores the goal postion

	double mDistanceToGoal;							/// Distance to the goal
	double mDistanceTolerance;						/// Distance tolerance to the goal

	Emotions mEmotionalState;						/// Keeps track of Nero's current emotion
	std::string mSpeech;							/// Keeps track of what is said by the user
	int mArousal;									/// Keeps track of the arousal contained in the speech of user
	bool mWakeUp;									/// Keeps track of whether Nero is listening to commands or not
	Locks mLock;									/// Checks whether the Controller is blocked or not and keeps track of which node it is being blocked by

	bool mGripperStop;

public:
	Controller() : mNodeHandle(""){}

	virtual ~Controller(){ mNodeHandle.shutdown();}

	void init();
	void updateRobotPosition();
	void moveArm(double x, double z);
	void moveArm(const geometry_msgs::PoseStamped msg);
	void moveBase(geometry_msgs::PoseStamped &stamped_goal);
	void moveHead(double x, double z);
	bool findObject(int object_id, geometry_msgs::PoseStamped &object_pose, float &min_y);
	void setFocusFace(bool active);
	void rotateBase(float angle);
	void positionBase(float dist);
	double positionBaseSpeed(double time, double lin_speed);
	void setGripper(bool open);
	void expressEmotion(uint8_t emotion);

	uint8_t get(int object, uint8 attempt_nr = 1);
	uint8_t wakeUp();
	uint8_t sleep();
	uint8_t	release();
	uint8_t respond();

	void navigationStateCB(const std_msgs::UInt8& msg);
	void baseGoalCB(const std_msgs::Float32& msg);
	void speechCB(const audio_processing::speech& msg);
	void headSpeedCB(const head::PitchYaw &msg);
	void baseSpeedCB(const geometry_msgs::Twist &msg);
	void armSpeedCB(const arm::armJointPos &msg);
	void objectPositionCB(const geometry_msgs::PoseStamped& msg);
	void gripperStateCB(const std_msgs::UInt8 &msg);

	void waitForLock();

	inline void convertQuaternion(geometry_msgs::Quaternion &q1, tf::Quaternion q2)
	{
		q1.x = q2.getX();
		q1.y = q2.getY();
		q1.z = q2.getZ();
		q1.w = q2.getW();
	}

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
};

#endif /* CONTROLLER_H_ */
