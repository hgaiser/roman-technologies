/*
 * AutonomeMobileController.cpp
 *
 *  Created on: 2012-01-20
 *      Author: wilson
 */
#include "mobile_base/AutonomeMobileController.h"

/**
 * Calculates a distance from point p to the line described by points a and b.
 */
double distanceToLine(geometry_msgs::Point p, geometry_msgs::Point a, geometry_msgs::Point b)
{
	double xu = p.x - a.x;
	double yu = p.y - a.y;
	double xv = b.x - a.x;
	double yv = b.y - a.y;
	if (xu * xv + yu * yv < 0)
		return sqrt( (p.x-a.x)*(p.x-a.x) + (p.y-a.y)*(p.y-a.y));

	xu = p.x - b.x;
	yu = p.y - b.y;
	xv = -xv;
	yv = -yv;
	if (xu * xv + yu * yv < 0)
		return sqrt( (p.x-b.x)*(p.x-b.x) + (p.y-b.y)*(p.y-b.y) );

	return fabs( (p.x*(a.y - b.y) + p.y*(b.x - a.x) + (a.x * b.y - b.x * a.y)) / sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y)));
}

float distanceBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b)
{
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	float dz = a.z - b.z;
	return sqrtf(dx*dx + dy*dy + dz*dz);
}

/**
 * Constructor
 */
AutonomeMobileController::AutonomeMobileController() : mNodeHandle(""), mFollowState(FOLLOW_STATE_IDLE)
{
	std::string pathTopic, speedFeedbackTopic, goalTopic, pathLengthTopic, followStateTopic;
	mNodeHandle.param<std::string>("path_topic", pathTopic, "/global_path");
	mNodeHandle.param<std::string>("speed_feedback_topic", speedFeedbackTopic, "/speedFeedbackTopic");
	mNodeHandle.param<std::string>("goal_topic", goalTopic, "/move_base_simple/goal");
	mNodeHandle.param<std::string>("path_length_topic", pathLengthTopic, "/path_length");
	mNodeHandle.param<std::string>("follow_state_topic", followStateTopic, "/follow_state");
	mNodeHandle.param<int>("refresh_rate", mRefreshRate, 5);
	mNodeHandle.param<double>("min_angular_speed", mMinAngularSpeed, 0.1);
	mNodeHandle.param<double>("max_angular_speed", mMaxAngularSpeed, 0.4);
	mNodeHandle.param<double>("min_linear_speed", mMinLinearSpeed, 0.2);
	mNodeHandle.param<double>("max_linear_speed", mMaxLinearSpeed, 0.3);
	mNodeHandle.param<double>("angular_adjustment_speed", mAngularAdjustmentSpeed, 0.05);
	mNodeHandle.param<double>("final_yaw_tolerance", mFinalYawTolerance, 0.2);
	mNodeHandle.param<double>("yaw_tolerance", mYawTolerance, 0.25 * M_PI);
	mNodeHandle.param<double>("distance_tolerance", mDistanceTolerance, 0.2);
	mNodeHandle.param<double>("reset_distance_tolerance", mResetDistanceTolerance, 0.5);

	mPathSub = mNodeHandle.subscribe(pathTopic, 1, &AutonomeMobileController::pathCb, this);
	mCommandPub = mNodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	mGoalPub = mNodeHandle.advertise<geometry_msgs::PoseStamped>(goalTopic, 1);
	mPathLengthPub = mNodeHandle.advertise<std_msgs::Float32>(pathLengthTopic, 1);
	mPathLengthPub = mNodeHandle.advertise<std_msgs::UInt8>(followStateTopic, 1);
}


/**
 * Controls the speed of the motors based on the received twist message.
 */
void AutonomeMobileMotorHandler::scaleTwist(const geometry_msgs::Twist& msg)
{
	double converted_right	= ZERO_SPEED, converted_left = ZERO_SPEED;
	double left_speed 	 	= ZERO_SPEED;
	double right_speed 		= ZERO_SPEED;

	//Convert linear speed to motor speeds in [rad/s]
	if (left_speed == ZERO_SPEED && right_speed == ZERO_SPEED)
	{
		double vel_linear = msg.linear.x / WHEEL_RADIUS;
		double vel_angular = msg.angular.z * (BASE_RADIUS / WHEEL_RADIUS) * 2;

		converted_left  = vel_linear - 0.5*vel_angular;
		converted_right = vel_linear + 0.5*vel_angular;
	}

	//Calculations are in positive numbers because the minimum between speed and scaled speed is used
	left_speed 	= std::abs(converted_left);
	right_speed = std::abs(converted_right);

	left_speed      = scaleSpeed(left_speed, STANDARD_AVOIDANCE_IMPACT, FRONT_SIDES_AVOIDANCE_DISTANCE, mFrontRight, mFrontCenterRight);
	right_speed		= scaleSpeed(right_speed, STANDARD_AVOIDANCE_IMPACT, FRONT_SIDES_AVOIDANCE_DISTANCE, mFrontLeft, mFrontCenterLeft);

	//Check right and left side for walls and avoid them when needed.
	//Don't avoid the walls when distance to both sides are close to the robot
	if(!(mLeft < SIDES_AVOIDANCE_DISTANCE && mRight < SIDES_AVOIDANCE_DISTANCE))
	{
		left_speed	= scaleSpeed(left_speed, SIDES_AVOIDANCE_IMPACT, SIDES_AVOIDANCE_DISTANCE, mRight);
		right_speed	= scaleSpeed(right_speed, SIDES_AVOIDANCE_IMPACT, SIDES_AVOIDANCE_DISTANCE, mLeft);
	}
	//Avoidance rules when object is in front of robot
	if(mFrontLeftCenter < FRONT_AVOIDANCE_DISTANCE || mFrontRightCenter < FRONT_AVOIDANCE_DISTANCE)
	{
		//Turn to left when left side has more space than right side
		if(mLeft > mRight && mRight < SIDES_AVOIDANCE_DISTANCE)
			left_speed = scaleSpeed(left_speed, FRONT_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);

		//Turn to right when right side has more space than left side
		else if(mLeft < mRight && mLeft < SIDES_AVOIDANCE_DISTANCE)
			right_speed = scaleSpeed(right_speed, FRONT_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);

		//Turn right when object is more to the left of the robot
		else if(mFrontLeft < FRONT_AVOIDANCE_DISTANCE || mFrontCenterLeft < FRONT_AVOIDANCE_DISTANCE)
			right_speed = scaleSpeed(right_speed, FRONT_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);

		//Turn left when object is more to the right of the robot
		else if(mFrontRight < FRONT_AVOIDANCE_DISTANCE || mFrontCenterRight < FRONT_AVOIDANCE_DISTANCE)
			left_speed = scaleSpeed(left_speed, FRONT_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);

		//Turn right when robot is driving straight to a wall
		else if(mFrontCenterRight < FRONT_SIDES_AVOIDANCE_DISTANCE-20 && mFrontCenterLeft < FRONT_SIDES_AVOIDANCE_DISTANCE-20)
		{
			right_speed = scaleSpeed(right_speed, 0.5*FRONT_CENTER_AND_SIDES_AVOIDANCE_IMPACT, FRONT_SIDES_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);
		}
		else
		{
			//If there is space front right of the robot, then turn to that side. Otherwise, turn to the other side.
			if(mFrontRight > FRONT_AVOIDANCE_DISTANCE)
				right_speed = scaleSpeed(right_speed, FRONT_CENTER_AND_SIDES_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);
			else
				left_speed = scaleSpeed(right_speed, FRONT_CENTER_AND_SIDES_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);
		}
	}

	//Restore the direction of the speeds when all calculations are done
	left_speed = converted_left < ZERO_SPEED ? -left_speed : left_speed;
	right_speed = converted_right < ZERO_SPEED ? -right_speed : right_speed;

	//Don't go backwards when there is a wall
	if(msg.linear.x < 0 && mRearLeft < REAR_HALTING_DISTANCE && mRearRight < REAR_HALTING_DISTANCE)
	{
		left_speed = 0.0;
		right_speed = 0.0;
	}

	//ROS_INFO("left %f right %f", left_speed, right_speed);

	mRightMotor.setSpeed(right_speed);
	mLeftMotor.setSpeed(left_speed);

}

/**
 * Returns the yaw that needs to be turned.
 */
double AutonomeMobileController::calculateDiffYaw()
{
	if (getPathSize() == 0)
	{
		//ROS_INFO("No change yaw.");
		return 0.0; // no change needed
	}
	if (getPathSize() == 1)
	{
		//ROS_INFO("Final point yaw.");
		double yaw = tf::getYaw(mPath.front().orientation);
		btVector3 orientation = btVector3(cos(yaw), sin(yaw), 0.0).rotate(btVector3(0,0,1), -tf::getYaw(mRobotPosition.getRotation()));
		return atan2(orientation.getY(), orientation.getX());
	}

	//ROS_INFO("Target yaw.");

	double dx = getNextPoint().x - mRobotPosition.getOrigin().getX();
	double dy = getNextPoint().y - mRobotPosition.getOrigin().getY();
	btVector3 diff = btVector3(dx, dy, 0).rotate(btVector3(0,0,1), -tf::getYaw(mRobotPosition.getRotation()));
	return atan2(diff.getY(), diff.getX());
}

/**
 * Continue to the next waypoint.
 */
void AutonomeMobileController::continuePath()
{
	mOrigin = getNextPoint();

	mPath.pop_front();
	if (getPathSize() == 0)
	{
		mFollowState = FOLLOW_STATE_FINISHED;
		ROS_INFO("Goal reached.");
	}
	else
		mFollowState = FOLLOW_STATE_TURNING;

	ROS_INFO("Waypoint reached.");
	geometry_msgs::Twist msg;
	mCommandPub.publish(msg);

}

/**
 * Publishes the total distance of the remaining path
 */
void AutonomeMobileController::publishPathLength()
{
	if (mPathLengthPub.getNumSubscribers() == 0)
		return;

	geometry_msgs::Point a, b;
	a.x = mRobotPosition.getOrigin().getX();
	a.y = mRobotPosition.getOrigin().getY();
	a.z = mRobotPosition.getOrigin().getZ();
	std_msgs::Float32 msg;
	for (std::list<geometry_msgs::Pose>::iterator i = mPath.begin(); i != mPath.end(); i++)
	{
		msg.data += distanceBetweenPoints(a, i->position);
	}
	mPathLengthPub.publish(msg);
}

/**
 * Publishes the robot state
 */
void AutonomeMobileController::publishState()
{
	if (mFollowStatePub.getNumSubscribers())
	{
		std_msgs::UInt8 msg;
		msg.data = mFollowState;
		mFollowStatePub.publish(msg);
	}

	// only send this state once
	if (mFollowState == FOLLOW_STATE_FINISHED)
		mFollowState = FOLLOW_STATE_IDLE;
}

/**
 * Clears path (continously calls continuePath until empty)
 */
void AutonomeMobileController::clearPath()
{
	while (getPathSize())
		continuePath();
}

/**
 * Updates path logic.
 */
void AutonomeMobileController::handlePath()
{
	geometry_msgs::Twist command;

	publishState();

	// no path? nothing to handle
	if (getPathSize() == 0)
		return;

	// update our position if possible.
	if (updateCurrentPosition() == false)
		return;

	double robotYaw = tf::getYaw(mRobotPosition.getRotation()); // only used for printing
	double diffYaw = calculateDiffYaw();

	ROS_INFO("Follow state: %d Robot pos: (%lf, %lf, %lf). Target pos: (%lf, %lf, %lf). RobotYaw: %lf. FocusYaw: %lf", mFollowState, mRobotPosition.getOrigin().getX(),
			mRobotPosition.getOrigin().getY(), mRobotPosition.getOrigin().getZ(), getNextPoint().x,
			getNextPoint().y, getNextPoint().z, robotYaw, diffYaw);

	geometry_msgs::Point robotPos;
	robotPos.x = mRobotPosition.getOrigin().x();
	robotPos.y = mRobotPosition.getOrigin().y();

	// check if we are too far away from our path. If so, reset our path.
	double distToPath = distanceToLine(robotPos, getOrigin(), getNextPoint());
	if (distToPath > mResetDistanceTolerance)
	{
		ROS_INFO("Too far away from path, re-publishing goal, getting new path.");
		geometry_msgs::PoseStamped goal;
		goal.pose = getGoal();
		goal.header.stamp = ros::Time::now();
		mGoalPub.publish(goal);
		clearPath();
	}

	// Did we turn away too much? Set state back to turning so we can correct it.
	if (fabs(diffYaw) > mYawTolerance)
	{
		ROS_INFO("Yaw too far away from target, turning back.");
		mFollowState = FOLLOW_STATE_TURNING;
	}

	switch (mFollowState)
	{
	case FOLLOW_STATE_TURNING:
		// Are we done turning, then change state to move forward
		if (fabs(diffYaw) < mFinalYawTolerance)
		{
			mFollowState = FOLLOW_STATE_FORWARD;
			break;
		}

		// if we're already near our goal point and not rotating for final orientation, continue the path
		if (getPathSize() != 1 && reachedNextPoint())
		{
			continuePath();
			break;
		}

		command.angular.z = diffYaw > 0.0 ? getScaledAngularSpeed(diffYaw) : -getScaledAngularSpeed(diffYaw);
		break;
	case FOLLOW_STATE_FORWARD:
		//ROS_INFO("Moving.");

		// Did we reach our waypoint? Then continue to the next waypoint.
		if (reachedNextPoint())
		{
			continuePath();
			break;
		}

		command.linear.x = getScaledLinearSpeed();
		// minor angular adjustments during forward movement.
		command.angular.z = diffYaw > 0.0 ? mAngularAdjustmentSpeed : -mAngularAdjustmentSpeed;
		break;
	default:
		break;
	}

	publishPathLength();
	mCommandPub.publish(command);
}

/**
 * Updates current position of the robot.
 */
bool AutonomeMobileController::updateCurrentPosition()
{
	// get current position in the map
	try
	{
		mTransformListener.lookupTransform("/map", "/base_link", ros::Time(0), mRobotPosition);
	}
	catch (tf::TransformException ex)
	{
		//ROS_ERROR("%s",ex.what());
		return false;
	}

	return true;
}

/**
 * Main follow loop.
 */
void AutonomeMobileController::spin()
{
	ros::Rate refreshRate(mRefreshRate);

	while (ros::ok())
	{
		handlePath();

		refreshRate.sleep();
		ros::spinOnce();
	}
}

/**
 * Receives path from PathPlanner and starts moving towards goal.
 */
void AutonomeMobileController::pathCb(const nav_msgs::Path &path)
{
	ROS_INFO("Received new path.");

	mPath.clear();

	for (size_t i = 0; i < path.poses.size(); i++)
		mPath.push_back(path.poses[i].pose);

	// add last point twice for orientation
	mPath.push_back(path.poses[path.poses.size() - 1].pose);

	mFollowState = FOLLOW_STATE_TURNING;
	// first waypoint is usually the starting position
	if (reachedNextPoint())
		continuePath();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "AutonomeMobileController");

	AutonomeMobileController autonomeMobileController;

	autonomeMobileController.spin();
	return 0;
}
