/*
 * PathFollower.cpp
 *
 *  Created on: Dec 20, 2011
 *      Author: hans
 */

#include "logical_unit/PathFollower.h"

PathFollower::PathFollower() : mFollowState(FOLLOW_STATE_IDLE), mAllowState(FOLLOW_STATE_IDLE)
{
	std::string pathTopic, speedFeedbackTopic;
	mNodeHandle.param<std::string>("path_topic", pathTopic, "/global_path");
	mNodeHandle.param<std::string>("speed_feedback_topic", speedFeedbackTopic, "/speedFeedbackTopic");
	mNodeHandle.param<int>("refresh_rate", mRefreshRate, 5);
	mNodeHandle.param<double>("yaw_tolerance", mYawTolerance, 0.1);
	mNodeHandle.param<double>("angular_speed", mAngularSpeed, 0.2);
	mNodeHandle.param<double>("linear_speed", mLinearSpeed, 0.2);
	mNodeHandle.param<double>("disable_transition_threshold", mDisableTransitionThreshold, 0.05);
	mNodeHandle.param<double>("final_yaw_tolerance", mFinalYawTolerance, 0.1);
	mNodeHandle.param<double>("distance_tolerance", mDistanceTolerance, 0.2);

	mPathSub = mNodeHandle.subscribe(pathTopic, 1, &PathFollower::pathCb, this);
	mSpeedFeedbackSub = mNodeHandle.subscribe(speedFeedbackTopic, 1, &PathFollower::speedCb, this);
	mCommandPub = mNodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

double PathFollower::calculateFocusYaw()
{
	if (getPathSize() == 0)
	{
		//ROS_INFO("No change yaw.");
		return tf::getYaw(mRobotPosition.getRotation()); // no change needed
	}
	if (getPathSize() == 1)
	{
		//ROS_INFO("Final point yaw.");
		return tf::getYaw(mPath.front().orientation);
	}

	//ROS_INFO("Target yaw.");

	/*btVector3 robot(mRobotPosition.getOrigin().getX(), mRobotPosition.getOrigin().getY(), mRobotPosition.getOrigin().getZ());
	btVector3 target(getNextPoint().x, getNextPoint().y, getNextPoint().z);
	return target.y() < robot.y() ? -robot.angle(target) : robot.angle(target);*/

	double dx = getNextPoint().x - mRobotPosition.getOrigin().getX();
	double dy = getNextPoint().y - mRobotPosition.getOrigin().getY();
	if (dx == 0.0)
		dx = 0.01;
	//ROS_INFO("dx: %lf, dy: %lf", dx, dy);
	double requiredYaw = atan(fabs(dy / dx));
	if (dx < 0.0)
	{
		if (dy < 0.0)
			requiredYaw = -M_PI + requiredYaw;
		else
			requiredYaw = M_PI - requiredYaw;
	}
	else if (dy < 0.0)
		requiredYaw = -1 * requiredYaw;
	return requiredYaw;
}

void PathFollower::continuePath()
{
	mPath.pop_front();
	if (getPathSize() == 0)
		ROS_INFO("Goal reached.");

	geometry_msgs::Twist msg;
	mCommandPub.publish(msg);
	mFollowState = FOLLOW_STATE_IDLE;
}

void PathFollower::handlePath(tf::TransformListener *transformListener)
{
	geometry_msgs::Twist command;

	// no path? nothing to handle
	if (getPathSize() == 0)
		return;

	// get current position in the map
	try
	{
		transformListener->lookupTransform("/map", "/base_link", ros::Time(0), mRobotPosition);
	}
	catch (tf::TransformException ex)
	{
		//ROS_ERROR("%s",ex.what());
	}

	double robotYaw = tf::getYaw(mRobotPosition.getRotation());
	double focusYaw = calculateFocusYaw();

	ROS_INFO("Robot pos: (%lf, %lf, %lf). Target pos: (%lf, %lf, %lf). RobotYaw: %lf. FocusYaw: %lf. Diff: %lf", mRobotPosition.getOrigin().getX(),
			mRobotPosition.getOrigin().getY(), mRobotPosition.getOrigin().getZ(), getNextPoint().x,
			getNextPoint().y, getNextPoint().z, robotYaw, focusYaw, focusYaw - robotYaw);

	if (fabs(focusYaw - robotYaw) > mYawTolerance)
	{
		if (canTurn())
		{
			//ROS_INFO("Turning.");
			mFollowState = FOLLOW_STATE_TURNING;
			command.angular.z = mAngularSpeed;
			if (focusYaw - robotYaw < 0.0)
				command.angular.z = -command.angular.z;
		}
		else
			ROS_INFO("Blocked turning.");
	}
	else
	{
		// small hack, last point is only for orientation, rarely gets here
		if (getPathSize() == 1)
			reachedNextPoint();

		if (canMove())
		{
			//ROS_INFO("Moving.");
			mFollowState = FOLLOW_STATE_FORWARD;

			if (reachedNextPoint())
				continuePath();
			else
				command.linear.x = mLinearSpeed;
		}
		else
			ROS_INFO("Blocked moving.");
	}

	mCommandPub.publish(command);
}

void PathFollower::spin()
{
	tf::TransformListener transformListener;
	ros::Rate refreshRate(mRefreshRate);

	while (ros::ok())
	{
		handlePath(&transformListener);

		refreshRate.sleep();
		ros::spinOnce();
	}
}

void PathFollower::pathCb(const nav_msgs::Path &path)
{
	ROS_INFO("Received new path.");

	mPath.clear();

	for (size_t i = 0; i < path.poses.size(); i++)
		mPath.push_back(path.poses[i].pose);

	// add last point twice for orientation
	mPath.push_back(path.poses[path.poses.size() - 1].pose);

	// first waypoint is usually the starting position
	if (reachedNextPoint())
		continuePath();
}

void PathFollower::speedCb(const geometry_msgs::Twist &twist)
{
	double vel_left = twist.linear.x + twist.angular.z / 2;
	double vel_right = twist.linear.x - twist.angular.z / 2;

	if ((fabs(vel_left) > mDisableTransitionThreshold || fabs(vel_right) > mDisableTransitionThreshold) &&
			mFollowState != FOLLOW_STATE_IDLE)
		mAllowState = mFollowState;
	else
		mAllowState = FOLLOW_STATE_IDLE;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PathFollower");

	PathFollower pf;

	pf.spin();
	return 0;
}