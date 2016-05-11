/*
 * affw_traj.cpp
 *
 *  Created on: 24.02.2016
 *      Author: NicolaiO
 */

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <fstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool moveTo(geometry_msgs::Pose& targetPose)
{
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose = targetPose;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO_STREAM("Init pose reached: " << targetPose);
		return true;
	}
	ROS_INFO("Could not reach target pose.");
	return false;
}

bool executeTraj(std::vector<geometry_msgs::TwistStamped>& setpoints, ros::Publisher& pub_set_vel)
{
	std::cout << "a" << std::endl;
	ros::Duration tStart(ros::Time::now().toSec());
	std::cout << "b" << std::endl;
	std::vector<geometry_msgs::TwistStamped>::iterator it = setpoints.begin();
	do {
		std::cout << "c" << std::endl;
		geometry_msgs::TwistStamped vel = *it;
		std::cout << vel.header.stamp << std::endl;
		vel.header.stamp += tStart;
		std::cout << "d" << std::endl;
		pub_set_vel.publish(vel);
		ros::spinOnce();
		std::cout << "e" << std::endl;
		it++;
		if(it < setpoints.end())
		{
			ros::Time tNext = it->header.stamp;
			ros::Duration diff = tNext - ros::Time::now();
			if(diff.toSec() <= 0 && diff.toSec() > -1e-3)
			{
				continue;
			} else if(diff.toSec() < 0)
			{
				ROS_WARN_STREAM("Negative sleep duration: " << diff.toSec());
			} else {
				diff.sleep();
			}
		}
		else
		{
			return true;
		}
	} while(ros::ok());

	return false;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "affw_traj");
	ros::NodeHandle n;
	ros::Publisher pub_set_vel = n.advertise<geometry_msgs::TwistStamped>(
			"/affw_ctrl/target_vel", 1);

	if(argc != 2)
	{
		std::cout << "Usage: " << argv[0] << " <traj-file>" << std::endl;
		return 1;
	}

	const char* filename = argv[1];
	std::ifstream file(filename);
	std::vector<geometry_msgs::TwistStamped> setpoints;

	double t, vx, vy, vw;
	while (file >> t >> vx >> vy >> vw) {
		geometry_msgs::TwistStamped vel;
		vel.header.stamp = ros::Time(t);
		vel.header.frame_id = "base_link";
		vel.twist.linear.x = vx;
		vel.twist.linear.y = vy;
		vel.twist.angular.z = vw;
		setpoints.push_back(vel);
	}
	file.close();

	while(ros::Time::now().isZero());
	ros::Duration(0.5).sleep();

	int numIterations = 1;
	double initPosW = 0;
	geometry_msgs::Pose initPose;
	ros::param::get("numIterations", numIterations);
	ros::param::get("initPosX", initPose.position.x);
	ros::param::get("initPosY", initPose.position.x);
	ros::param::get("initPosW", initPosW);
	initPose.orientation = tf::createQuaternionMsgFromYaw(initPosW);

	for(int i = 0;i < numIterations;i++)
	{
		ROS_INFO("Start iteration %d", i);
		if(!moveTo(initPose))
		{
			break;
		}
		ROS_INFO("Start trajectory");
		if(!executeTraj(setpoints, pub_set_vel))
		{
			break;
		}
	}

	ros::shutdown();

	return 0;
}

