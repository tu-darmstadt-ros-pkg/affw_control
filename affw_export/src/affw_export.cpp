/*
 * affw_export.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include <affw_msgs/ProcTime.h>
#include <affw_msgs/TargetRequest.h>
#include <boost/filesystem/operations.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

std::string folder = "/tmp/affw_data";
std::ofstream fAffw, fProcTime;
bool first = true;

void callbackAffw(const affw_msgs::TargetRequest::ConstPtr& tr) {

	if(first)
	{
		std::ofstream fDim(std::string(folder + "/dimensions.csv").c_str());
		fDim << tr->state.size() << " " << tr->target.size() << std::endl;
		fDim.close();
		first = false;
	}

	fAffw << std::fixed << std::setw(11) << std::setprecision(6)
			<< tr->header.stamp.toSec();

	for (int i = 0; i < tr->state.size(); i++)
		fAffw << " " << tr->state[i];
	for (int i = 0; i < tr->target.size(); i++)
		fAffw << " " << tr->target[i];
	for (int i = 0; i < tr->action.size(); i++)
		fAffw << " " << tr->action[i];
	for (int i = 0; i < tr->actionComp.size(); i++)
		fAffw << " " << tr->actionComp[i];

	fAffw << " " << tr->timeAction2StateOffset << " " << tr->timeStateDelayOffset;

	fAffw << std::endl;
}

void callbackProcTime(const affw_msgs::ProcTime::ConstPtr& proc) {

	fProcTime << std::fixed << std::setw(11) << std::setprecision(6)
			<< proc->stamp.toSec() << " " << (int) (proc->type) << " "
			<< proc->procTime << std::endl;
}

void openFiles()
{
	// close log files
	if(fAffw.is_open()) fAffw.close();
	if(fProcTime.is_open()) fProcTime.close();

	if(folder.empty())
	{
		return;
	}

	std::string affwFile = folder + "/affw.csv";
	std::string procTimeFile = folder + "/procTime.csv";

	// create folder if it does not exist
	boost::filesystem::create_directories(folder);
	// remove old files if they exist
	boost::filesystem::remove(affwFile);
	boost::filesystem::remove(procTimeFile);


	// open log files
	fAffw.open(affwFile.c_str());
	fProcTime.open(procTimeFile.c_str());

	first = true;
}

void callbackDataFolder(const std_msgs::String::ConstPtr& dataFolder)
{
	if(folder != dataFolder->data)
	{
		folder = dataFolder->data;
		openFiles();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "affw_export");
	ros::NodeHandle n;
	ros::param::get("dataFolder", folder);
	bool split = false;
	ros::param::get("split", split);

	ros::Subscriber sub_dataFolder;
	if(split) {
		sub_dataFolder = n.subscribe("dataFolder", 1,	callbackDataFolder);
	} else {
		openFiles();
	}

	// subscribe to topics
	ros::Subscriber sub_affw = n.subscribe("/affw_ctrl/target_request", 50,
			callbackAffw);
	ros::Subscriber sub_procTime = n.subscribe("/affw_ctrl/proc_time", 50,
			callbackProcTime);

	// go
	ros::spin();

	// close log files
	fAffw.close();
	fProcTime.close();

	return 0;
}

