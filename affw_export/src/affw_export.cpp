/*
 * affw_export.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include <affw_msgs/ProcTime.h>
#include <affw_msgs/TargetRequest.h>
#include <boost/filesystem/operations.hpp>
#include <boost/thread.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <signal.h>
#include <tf/tf.h>

#define LOG_AFFW 0
#define LOG_PROC_TIME 1
#define LOG_STATE 2
#define LOG_ODOM 3
#define LOG_SET 4
#define NUM_LOGFILES 5

std::string logFiles[] = {"affw.csv","procTime.csv","globalState.csv", "localVel.csv", "set.csv"};
std::vector< std::vector<float> > data[NUM_LOGFILES];
std::vector< std::vector<float> > dataAll[NUM_LOGFILES];
std::string folder = "/tmp/affw_data";
std::string folderCustom;

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

void callbackAffw(const affw_msgs::TargetRequest::ConstPtr& tr) {

	if(data[LOG_AFFW].empty() && !folderCustom.empty())
	{
		std::ofstream fDim(std::string(folderCustom + "/dimensions.csv").c_str());
		fDim << tr->state.size() << " " << tr->target.size() << std::endl;
		fDim.close();
	}
	if(dataAll[LOG_AFFW].empty())
	{
		std::ofstream fDim(std::string(folder + "/dimensions.csv").c_str());
		fDim << tr->state.size() << " " << tr->target.size() << std::endl;
		fDim.close();
	}

	std::vector<float> ds;

	ds.push_back(tr->header.stamp.toSec());

	for (int i = 0; i < tr->state.size(); i++)
		ds.push_back(tr->state[i]);
	for (int i = 0; i < tr->target.size(); i++)
		ds.push_back(tr->target[i]);
	for (int i = 0; i < tr->action.size(); i++)
		ds.push_back(tr->action[i]);
	for (int i = 0; i < tr->actionComp.size(); i++)
		ds.push_back(tr->actionComp[i]);

	ds.push_back(tr->timeAction2StateOffset.toSec());
	ds.push_back(tr->timeStateDelayOffset.toSec());

	data[LOG_AFFW].push_back(ds);
	dataAll[LOG_AFFW].push_back(ds);
}

void callbackProcTime(const affw_msgs::ProcTime::ConstPtr& proc) {

	std::vector<float> ds;
	ds.push_back(proc->stamp.toSec());
	ds.push_back(proc->type);
	ds.push_back(proc->procTime);
	data[LOG_PROC_TIME].push_back(ds);
	dataAll[LOG_PROC_TIME].push_back(ds);
}

void callbackState(const nav_msgs::Odometry::ConstPtr& odom) {

	std::vector<float> ds;
	ds.push_back(odom->header.stamp.toSec());
	ds.push_back(odom->pose.pose.position.x);
	ds.push_back(odom->pose.pose.position.y);
	ds.push_back(getOrientation(odom->pose.pose.orientation));
	ds.push_back(odom->twist.twist.linear.x);
	ds.push_back(odom->twist.twist.linear.y);
	ds.push_back(odom->twist.twist.angular.z);
	data[LOG_STATE].push_back(ds);
	dataAll[LOG_STATE].push_back(ds);
}

void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom) {

	std::vector<float> ds;
	ds.push_back(odom->header.stamp.toSec());
	ds.push_back(odom->twist.twist.linear.x);
	ds.push_back(odom->twist.twist.linear.y);
	ds.push_back(odom->twist.twist.angular.z);
	data[LOG_ODOM].push_back(ds);
	dataAll[LOG_ODOM].push_back(ds);
}

void callbackSet(const geometry_msgs::Twist::ConstPtr& twist) {

	std::vector<float> ds;
	ds.push_back(ros::Time::now().toSec());
	ds.push_back(twist->linear.x);
	ds.push_back(twist->linear.y);
	ds.push_back(twist->angular.z);
	data[LOG_SET].push_back(ds);
	dataAll[LOG_SET].push_back(ds);
}

void save(std::string& folder, std::string& logFile, std::vector<std::vector<float> >& dat)
{
	std::string filepath = folder + "/" + logFile;
	boost::filesystem::remove(filepath);
	std::ofstream f;
	f.open(filepath.c_str());
	f << std::fixed << std::setw(11) << std::setprecision(6);
	for(std::vector<std::vector<float> >::iterator it = dat.begin(); it != dat.end(); it++)
	{
		std::vector<float> ds = *it;
		for(std::vector<float>::iterator it2 = ds.begin(); it2 != ds.end(); it2++)
		{
			f << *it2 << " ";
		}
		f << std::endl;
	}
	f.close();
}

void saveAll()
{
	if(!folderCustom.empty())
	{
		for(int i=0;i<NUM_LOGFILES;i++)
		{
			save(folderCustom, logFiles[i], data[i]);
			data[i].clear();
		}
	}
}

void mySigintHandler(int sig)
{
	saveAll();

	for(int i=0;i<NUM_LOGFILES;i++)
	{
		save(folder, logFiles[i], dataAll[i]);
	}

	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}


void changeDataFolder(std::string dataFolder)
{
	saveAll();

	folderCustom = dataFolder;
	if(!folderCustom.empty())
		boost::filesystem::create_directories(folderCustom);
	for(int i=0;i<NUM_LOGFILES;i++)
		data[i].clear();
}

void callbackDataFolder(const std_msgs::String::ConstPtr& dataFolder)
{
	if(folderCustom != dataFolder->data)
	{
		changeDataFolder(dataFolder->data);
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "affw_export");
	ros::NodeHandle n;
	ros::param::get("dataFolder", folder);
	bool split = false;
	ros::param::get("split", split);

	ros::Subscriber sub_dataFolder = n.subscribe("dataFolder", 1, callbackDataFolder);

	// subscribe to topics
	ros::Subscriber sub_affw = n.subscribe("/affw_ctrl/target_request", 50,
			callbackAffw);
	ros::Subscriber sub_procTime = n.subscribe("/affw_ctrl/proc_time", 50,
			callbackProcTime);
	ros::Subscriber sub_odom = n.subscribe("odom", 50, callbackOdom);
	ros::Subscriber sub_state = n.subscribe("state", 50, callbackState);
	ros::Subscriber sub_set = n.subscribe("cmd_vel", 50, callbackSet);

	signal(SIGINT, mySigintHandler);

	// go
	ros::spin();

	return 0;
}

