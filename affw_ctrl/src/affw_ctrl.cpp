/*
 * affw_ctrl.cpp
 *
 *  Created on: 24.02.2016
 *      Author: NicolaiO
 */

#include <affw_msgs/ActionRequest.h>
#include <affw_msgs/ActionRequestRequest.h>
#include <affw_msgs/State.h>
#include <affw_msgs/TargetRequest.h>
#include <affw_msgs/ProcTime.h>

#include <boost/bind/arg.hpp>
#include <boost/bind/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/filesystem.hpp>
#include <message_filters/connection.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <stddef.h>
#include <iostream>
#include <queue>
#include <vector>
#include <chrono>

#include "learner/Config.h"
#include "learner/DummyLearner.h"
#include "learner/LWPRLearner.h"
#include "learner/ModelLearner.h"

// publishers
ros::Publisher targetRequest_pub;
ros::Publisher processing_time_pub;

// states
std::vector<double> curState;
affw::ModelLearner* learner = NULL;
affw::Config config;

// configs
ros::Duration timeOffset(0.05);
bool updateModel = true;
int nFrames = 1;
std::string learner_type = "";


bool actionRequest(affw_msgs::ActionRequest::Request &req,
		affw_msgs::ActionRequest::Response &res) {

	affw_msgs::TargetRequest tr;
	tr.header = req.header;
	tr.header.stamp += timeOffset;
	tr.state.insert(tr.state.end(), curState.begin(), curState.end());
	tr.target = req.setPoint;
	tr.action = req.setPoint;

	if(curState.size() == nFrames * tr.action.size())
	{
		affw::Vector s(tr.state.begin(), tr.state.end());
		affw::Vector t(tr.target.begin(), tr.target.end());

		// get action compensation
		std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		affw::Vector actionComp = learner->getActionCompensation(s, t);
		std::chrono::high_resolution_clock::time_point stop = std::chrono::high_resolution_clock::now();
	    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( stop - start ).count();
		affw_msgs::ProcTime proc;
		proc.stamp = tr.header.stamp;
		proc.type = 1;
		proc.procTime = duration;
		processing_time_pub.publish(proc);

		tr.actionComp.reserve(actionComp.size());
		for(int i=0;i<actionComp.size(); i++)
		{
			tr.actionComp.push_back(actionComp[i]);
			tr.action[i] += actionComp[i];
		}
		targetRequest_pub.publish(tr);
		ros::spinOnce();
	}

	res.outVel.insert(res.outVel.end(), tr.action.begin(), tr.action.end());
	assert(res.outVel.size() == req.setPoint.size());

	return true;
}

void saveModelCallback(const std_msgs::String::ConstPtr& location) {
	if(learner != NULL)
	{
		learner->write(location->data);
	}
}

void feedbackStateCallback(const affw_msgs::State::ConstPtr& state) {
	int dim = state->vel.size();
	config.setInt("actionDim", dim);
	config.setInt("nFrames", nFrames);

	if(learner == NULL)
	{
		if(learner_type == "lwpr") {
			learner = new affw::LWPR_Learner(&config);
			std::cout << "LWPR learner created" << std::endl;
		} else
		{
			learner = new affw::DummyLearner();
			std::cout << "dummy learner created" << std::endl;
		}
	}

	std::vector<double> newState;
	if(curState.size() >= nFrames * dim)
	{
		newState.insert(newState.end(), curState.begin() + dim, curState.end());
	} else
	{
		newState.insert(newState.end(), curState.begin(), curState.end());
	}
	newState.insert(newState.end(), state->vel.begin(), state->vel.end());

	curState = newState;
}

void syncCallback(const affw_msgs::State::ConstPtr& state,
		const affw_msgs::TargetRequest::ConstPtr& target)
{
	// (state, target) -> action
	// (state, action) -> nextState
	int dim = state->vel.size();
	affw::Vector s,t(dim),a(dim),ac(dim),ns(dim);

	s.insert(s.end(), target->state.begin(), target->state.end());

	for(int i=0;i<dim;i++)
	{
		t[i] = target->target[i];
		a[i] = target->action[i];
		ac[i] = target->actionComp[i];
		ns[i] = state->vel[i];
	}

	std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
	if(updateModel)	learner->addData(s,t,a,ac,ns);
	std::chrono::high_resolution_clock::time_point stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( stop - start ).count();
	affw_msgs::ProcTime proc;
	proc.stamp = target->header.stamp;
	proc.type = 2;
	proc.procTime = duration;
	processing_time_pub.publish(proc);
}

void updateModelCallback(const std_msgs::Bool::ConstPtr& enabled)
{
	updateModel = enabled->data;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "affw_ctrl");
	ros::NodeHandle n;

	std::string dataFolder = "/tmp/affw_data";
	bool reset = false;
	ros::param::get("learner", learner_type);
	ros::param::get("dataFolder", dataFolder);
	ros::param::get("reset", reset);

	if(reset)
	{
		ROS_INFO("Clearing data folder");
		bool removed = boost::filesystem::remove_all(dataFolder);
		if(!removed)
			ROS_ERROR_STREAM("Could not clear data folder: " << dataFolder);
	}

	// create dataFolder, if it does not exist
	boost::filesystem::create_directories(dataFolder);

	// read config, if it exists
	std::string configName = dataFolder + "/affw_" + learner_type + ".cfg";
	if(boost::filesystem::exists(configName))
	{
		config.read(configName.c_str());
		ROS_INFO_STREAM("Read affw config: " << configName);
	}
	config.setString("dataFolder", dataFolder);
	updateModel = config.getBool("updateModel", true);
	timeOffset = ros::Duration(config.getDouble("timeOffset", 0.1));
	nFrames = config.getInt("nFrames", nFrames);

	// debugging:
	std::cout << "Current config:" << std::endl;
	config.print();

	// set up state synchronization
	message_filters::Subscriber<affw_msgs::TargetRequest> targetRequest_sub(n, "/affw_ctrl/target_request", 1);
	message_filters::Subscriber<affw_msgs::State> state_sub(n, "/affw_ctrl/state", 1);
	typedef message_filters::sync_policies::ApproximateTime<affw_msgs::State, affw_msgs::TargetRequest> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), state_sub, targetRequest_sub);
	sync.registerCallback(boost::bind(&syncCallback, _1, _2));

	// set up topics
	targetRequest_pub = n.advertise<affw_msgs::TargetRequest>("/affw_ctrl/target_request", 1);
	processing_time_pub = n.advertise<affw_msgs::ProcTime>("/affw_ctrl/proc_time", 1);
	ros::ServiceServer service = n.advertiseService("/affw_ctrl/action", actionRequest);
	ros::Subscriber sub_fdbk_vel = n.subscribe("/affw_ctrl/state", 1, feedbackStateCallback);
	ros::Subscriber sub_save = n.subscribe("/affw_ctrl/save_model", 1, saveModelCallback);
	ros::Subscriber sub_updateModel = n.subscribe("/affw_ctrl/update_model", 1, updateModelCallback);

	// gogogo
	ros::spin();

	// save data
	config.setBool("updateModel", updateModel);
	if(learner != NULL) learner->write(dataFolder);
	if(config.write(configName.c_str()))
	{
		std::cout << "Wrote affw config to " << configName << std::endl;
	} else {
		std::cout << "Could not write affw config to " << configName << std::endl;
	}

	return 0;
}

