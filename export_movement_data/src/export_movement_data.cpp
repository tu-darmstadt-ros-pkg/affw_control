/*
 * export_data.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include <boost/filesystem/operations.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

std::ofstream fGlobalState, fLocalVel, fSet;

double getOrientation(geometry_msgs::Quaternion gq) {
	double ow = gq.w;
	double ox = gq.x;
	double oy = gq.y;
	double oz = gq.z;

	tf::Quaternion q(ox, oy, oz, ow);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}

void callbackState(const nav_msgs::Odometry::ConstPtr& odom) {

	fGlobalState
		<< odom->header.stamp.toSec() << " "
		<< odom->pose.pose.position.x << " "
		<< odom->pose.pose.position.y << " "
		<< getOrientation(odom->pose.pose.orientation) << " "
		<< odom->twist.twist.linear.x << " "
		<< odom->twist.twist.linear.y << " "
		<< odom->twist.twist.angular.z << std::endl;
}

void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom) {

	fLocalVel
		<< odom->header.stamp.toSec() << " "
		<< odom->twist.twist.linear.x << " "
		<< odom->twist.twist.linear.y << " "
		<< odom->twist.twist.angular.z << std::endl;
}

void callbackSet(const geometry_msgs::Twist::ConstPtr& twist) {

	fSet 	<< ros::Time::now().toSec() << " "
			<< twist->linear.x << " "
			<< twist->linear.y << " "
			<< twist->angular.z << std::endl;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "export_movement_data");
	ros::NodeHandle n;

	// wait for valid time
	while(ros::Time::now().isZero());

	std::string folder = "/tmp/ros_movement_data";
	ros::param::get("dataFolder", folder);

	std::string localVelFile = folder + "/localVel.csv";
	std::string globalStateFile = folder + "/globalState.csv";
	std::string setpointFile = folder + "/set.csv";

	// create folder if it does not exist
	boost::filesystem::create_directories(folder);
	// remove old files if they exist
	boost::filesystem::remove(localVelFile);
	boost::filesystem::remove(globalStateFile);
	boost::filesystem::remove(setpointFile);

	fLocalVel.open(localVelFile.c_str());
	fGlobalState.open(globalStateFile.c_str());
	fSet.open(setpointFile.c_str());

	fLocalVel << std::fixed << std::setw(11) << std::setprecision(6);
	fGlobalState << std::fixed << std::setw(11) << std::setprecision(6);
	fSet << std::fixed << std::setw(11) << std::setprecision(6);

	ros::Subscriber sub_odom = n.subscribe("odom", 1, callbackOdom);
	ros::Subscriber sub_state = n.subscribe("state", 1, callbackState);
	ros::Subscriber sub_set = n.subscribe("cmd_vel", 1, callbackSet);

	ros::spin();

	fLocalVel.close();
	fGlobalState.close();
	fSet.close();

	return 0;
}

