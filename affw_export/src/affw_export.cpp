/*
 * affw_export.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include <affw_msgs/ProcTime.h>
#include <affw_msgs/TargetRequest.h>
#include <boost/filesystem.hpp>
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
#include <time.h>
#include <tf/tf.h>

#define LOG_AFFW 0
#define LOG_PROC_TIME 1
#define LOG_STATE 2
#define LOG_ODOM 3
#define LOG_SET 4
#define NUM_LOGFILES 5

std::string logFiles[] = {"affw.csv","procTime.csv","globalState.csv", "localVel.csv", "set.csv"};
//std::vector< std::vector<double> > data[NUM_LOGFILES];
//std::vector< std::vector<double> > dataAll[NUM_LOGFILES];
std::string folder = "/tmp/affw_data";
std::string folderCustom;

bool first = true;
bool firstCustom = true;

std::ofstream ofStreams[2][NUM_LOGFILES];

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

void saveDataset(int logId, std::vector<double>& ds)
{
	for(int folderId = 0;folderId < 2; folderId++)
	{
		if(ofStreams[folderId][logId].is_open())
		{
			for(std::vector<double>::iterator it2 = ds.begin(); it2 != ds.end(); it2++)
			{
				ofStreams[folderId][logId]  << *it2 << " ";
			}
			ofStreams[folderId][logId]  << std::endl;
		}
	}
}

void callbackAffw(const affw_msgs::TargetRequest::ConstPtr& tr) {

	double dim = tr->target.size();
	double stateSize = tr->state.size();
	if(firstCustom && !folderCustom.empty())
	{
		std::ofstream fDim(std::string(folderCustom + "/dimensions.csv").c_str());
		fDim << tr->state.size() << " " << tr->target.size() << std::endl;
		fDim.close();
		firstCustom = false;
	}
	if(first)
	{
		std::ofstream fDim(std::string(folder + "/dimensions.csv").c_str());
		fDim << tr->state.size() << " " << tr->target.size() << std::endl;
		fDim.close();
		first = false;
	}

	if(tr->action.size() != dim ||
			tr->actionComp.size() != dim ||
			tr->nextState.size() != dim ||
			tr->nextActionComp.size() != dim)
	{
		ROS_ERROR_STREAM("Invalid dimensions: "
				<< tr->action.size() << " "
				<< tr->actionComp.size() << " "
				<< tr->nextState.size() << " "
				<< tr->nextActionComp.size() << " ");
	}

	std::vector<double> ds;

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

	for (int i = 0; i < tr->nextState.size(); i++)
		ds.push_back(tr->nextState[i]);
	for (int i = 0; i < tr->nextActionComp.size(); i++)
		ds.push_back(tr->nextActionComp[i]);
	for (int i = 0; i < tr->learnerDebug.size(); i++)
		ds.push_back(tr->learnerDebug[i]);

//	data[LOG_AFFW].push_back(ds);
//	dataAll[LOG_AFFW].push_back(ds);
	saveDataset(LOG_AFFW, ds);
}

void callbackProcTime(const affw_msgs::ProcTime::ConstPtr& proc) {

	std::vector<double> ds;
	ds.push_back(proc->stamp.toSec());
	ds.push_back(proc->type);
	ds.push_back(proc->procTime);
//	data[LOG_PROC_TIME].push_back(ds);
//	dataAll[LOG_PROC_TIME].push_back(ds);
	saveDataset(LOG_PROC_TIME, ds);
}

void callbackState(const nav_msgs::Odometry::ConstPtr& odom) {

	std::vector<double> ds;
	ds.push_back(odom->header.stamp.toSec());
	ds.push_back(odom->pose.pose.position.x);
	ds.push_back(odom->pose.pose.position.y);
	ds.push_back(getOrientation(odom->pose.pose.orientation));
	ds.push_back(odom->twist.twist.linear.x);
	ds.push_back(odom->twist.twist.linear.y);
	ds.push_back(odom->twist.twist.angular.z);
//	data[LOG_STATE].push_back(ds);
//	dataAll[LOG_STATE].push_back(ds);
	saveDataset(LOG_STATE, ds);
}

void callbackPose(const geometry_msgs::PoseStamped::ConstPtr& odom) {

	std::vector<double> ds;
	ds.push_back(odom->header.stamp.toSec());
	ds.push_back(odom->pose.position.x);
	ds.push_back(odom->pose.position.y);
	ds.push_back(getOrientation(odom->pose.orientation));
	ds.push_back(0);
	ds.push_back(0);
	ds.push_back(0);
//	data[LOG_STATE].push_back(ds);
//	dataAll[LOG_STATE].push_back(ds);
	saveDataset(LOG_STATE, ds);
}

void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom) {

	std::vector<double> ds;
	ds.push_back(odom->header.stamp.toSec());
	ds.push_back(odom->twist.twist.linear.x);
	ds.push_back(odom->twist.twist.linear.y);
	ds.push_back(odom->twist.twist.angular.z);
//	data[LOG_ODOM].push_back(ds);
//	dataAll[LOG_ODOM].push_back(ds);
	saveDataset(LOG_ODOM, ds);
}

void callbackSet(const geometry_msgs::Twist::ConstPtr& twist) {

	std::vector<double> ds;
	ds.push_back(ros::Time::now().toSec());
	ds.push_back(twist->linear.x);
	ds.push_back(twist->linear.y);
	ds.push_back(twist->angular.z);
//	data[LOG_SET].push_back(ds);
//	dataAll[LOG_SET].push_back(ds);
	saveDataset(LOG_SET, ds);
}

void save(std::string& folder, std::string& logFile, std::vector<std::vector<double> >& dat)
{
	std::string filepath = folder + "/" + logFile;
	if(boost::filesystem::exists(filepath))
	{
		boost::filesystem::remove(filepath);
	}
	std::ofstream f;
	f.open(filepath.c_str());
	f << std::fixed << std::setw(11) << std::setprecision(6);
	for(std::vector<std::vector<double> >::iterator it = dat.begin(); it != dat.end(); it++)
	{
		std::vector<double> ds = *it;
		for(std::vector<double>::iterator it2 = ds.begin(); it2 != ds.end(); it2++)
		{
			f << *it2 << " ";
		}
		f << std::endl;
	}
	f.close();
}

//void saveAll()
//{
//	if(!folderCustom.empty())
//	{
//		for(int i=0;i<NUM_LOGFILES;i++)
//		{
//			save(folderCustom, logFiles[i], data[i]);
//			data[i].clear();
//		}
//	}
//}

void mySigintHandler(int sig)
{
//	saveAll();

	for(int i=0;i<NUM_LOGFILES;i++)
	{
//		save(folder, logFiles[i], dataAll[i]);

		ofStreams[0][i].close();
		ofStreams[1][i].close();
	}

	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}


void changeDataFolder(std::string dataFolder)
{
//	saveAll();

	folderCustom = dataFolder;
	if(!folderCustom.empty())
		boost::filesystem::create_directories(folderCustom);
	for(int i=0;i<NUM_LOGFILES;i++)
	{
//		data[i].clear();
		ofStreams[1][i].close();
		if(!folderCustom.empty())
		{
			std::string filepath = folderCustom + "/" + logFiles[i];
			ofStreams[1][i].open(filepath.c_str());
			if(!ofStreams[1][i].is_open())
				ROS_ERROR_STREAM("Could not open file: " << filepath);
			ofStreams[1][i] << std::fixed << std::setw(11) << std::setprecision(6);
		}
	}
	firstCustom = true;
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

	boost::filesystem::path p(folder);
	int i = 0;
	while(boost::filesystem::exists(p))
	{
		if(i > 5)
		{
			ROS_ERROR("Old data folder present. Exiting...");
			ros::Duration(0.5).sleep();
			return 1;
		}
		ros::Duration(0.1).sleep();
		i++;
	}

	boost::filesystem::create_directories(p);

	for(int i=0;i<NUM_LOGFILES;i++)
	{
		std::string filepath = folder + "/" + logFiles[i];
		ofStreams[0][i].open(filepath.c_str());
		if(!ofStreams[0][i].is_open())
			ROS_ERROR_STREAM("Could not open file: " << filepath);
		ofStreams[0][i] << std::fixed << std::setw(11) << std::setprecision(6);
	}

	std::string odom_topic = "odom";
	std::string state_topic = "state";
	std::string pose_topic = "";
	std::string cmd_vel_topic = "cmd_vel";
	ros::param::get("odom_topic", odom_topic);
	ros::param::get("state_topic", state_topic);
	ros::param::get("pose_topic", pose_topic);
	ros::param::get("cmd_vel_topic", cmd_vel_topic);

	ros::Subscriber sub_dataFolder = n.subscribe("dataFolder", 1, callbackDataFolder);

	// subscribe to topics
	ros::Subscriber sub_affw = n.subscribe("/affw_ctrl/target_request", 10,
			callbackAffw);
	ros::Subscriber sub_procTime = n.subscribe("/affw_ctrl/proc_time", 10,
			callbackProcTime);
	ros::Subscriber sub_odom = n.subscribe(odom_topic, 10, callbackOdom);
	ros::Subscriber sub_state;
	if(!state_topic.empty())
	{
		sub_state = n.subscribe(state_topic, 10, callbackState);
	}
	else if(!pose_topic.empty())
	{
		sub_state = n.subscribe(pose_topic, 10, callbackPose);
	}
	ros::Subscriber sub_set = n.subscribe(cmd_vel_topic, 10, callbackSet);

	signal(SIGINT, mySigintHandler);

	// go
	ros::spin();

	return 0;
}

