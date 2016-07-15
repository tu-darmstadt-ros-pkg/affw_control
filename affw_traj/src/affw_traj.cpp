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
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <fstream>
#include <signal.h>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher pub_cmd_vel;
ros::Publisher pub_set_vel;
ros::Publisher pub_dataFolder;
ros::Publisher pub_saveModel;
bool running = true;

bool moveTo(geometry_msgs::Pose& targetPose)
{
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose = targetPose;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Init pose reached.");
		return true;
	}
	ROS_INFO("Could not reach target pose.");
	return false;
}

bool executeTraj(std::vector<geometry_msgs::TwistStamped>& setpoints, ros::Publisher& pub_set_vel)
{
	ros::Duration tStart(ros::Time::now().toSec());
	std::vector<geometry_msgs::TwistStamped>::iterator it = setpoints.begin();
	do {
		geometry_msgs::TwistStamped vel = *it;
//		std::cout << vel.header.stamp << " " <<
//				vel.twist.linear.x << " " <<
//				vel.twist.linear.y << " " <<
//				vel.twist.angular.z << std::endl;
		vel.header.stamp += tStart;
		pub_set_vel.publish(vel);
		ros::spinOnce();
		it++;
		if(it < setpoints.end())
		{
			ros::Time tNext = it->header.stamp + tStart;
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
	} while(ros::ok() && running);

	return false;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "affw_traj");
	ros::NodeHandle n;

	std::string dataFolder = "/tmp/traj_data";
	ros::param::get("dataFolder", dataFolder);

	pub_set_vel = n.advertise<geometry_msgs::TwistStamped>(
			"/affw_ctrl/target_vel", 1);
	pub_dataFolder = n.advertise<std_msgs::String>("dataFolder", 1);
	ros::Publisher pub_updateModel = n.advertise<std_msgs::Bool>("/affw_ctrl/update_model", 1, true);
	pub_saveModel = n.advertise<std_msgs::String>("/affw_ctrl/save_model", 1);
	pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd/vel", 1);

	if(argc != 2)
	{
		std::cout << "Usage: " << argv[0] << " <traj-file>" << std::endl;
		return 1;
	}

	bool useInitPos = true;
	ros::param::get("useInitPos", useInitPos);

	int numIterations = 3;
	double initPosW = 0;
	geometry_msgs::Pose initPose;
	ros::param::get("numIterations", numIterations);
	ros::param::get("initPosX", initPose.position.x);
	ros::param::get("initPosY", initPose.position.y);
	ros::param::get("initPosW", initPosW);
	initPose.orientation = tf::createQuaternionMsgFromYaw(initPosW);

	int numEvalIterations = 0;
	ros::param::get("numEvalIterations", numEvalIterations);

	while(ros::Time::now().isZero());
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

	geometry_msgs::Twist zeroVel;
	pub_cmd_vel.publish(zeroVel);

	std_msgs::String dataFolderMsg;
	dataFolderMsg.data = "";
	pub_dataFolder.publish(dataFolderMsg);

	int i = 0;
	while(boost::filesystem::remove_all(dataFolder + "/iteration_" + boost::lexical_cast<std::string>(i++)));

	for(int i = 0;i < numIterations + numEvalIterations;i++)
	{
		ROS_INFO("Start iteration %d", i);
		if(useInitPos && !moveTo(initPose))
		{
			break;
		}

		std_msgs::Bool b;
		if(i >= numIterations)
		{
			b.data = false;
		} else {
			b.data = true;
		}
		pub_updateModel.publish(b);
		ros::spinOnce();

		ros::Duration(0.3).sleep();
		ROS_INFO("Start trajectory");
		dataFolderMsg.data = dataFolder + "/iteration_" + boost::lexical_cast<std::string>(i);
		pub_dataFolder.publish(dataFolderMsg);
		ros::spinOnce();
		if(!executeTraj(setpoints, pub_set_vel))
		{
			break;
		}
		pub_saveModel.publish(dataFolderMsg);
		dataFolderMsg.data = "";
		pub_dataFolder.publish(dataFolderMsg);
		ros::spinOnce();
	}



	return 0;
}

