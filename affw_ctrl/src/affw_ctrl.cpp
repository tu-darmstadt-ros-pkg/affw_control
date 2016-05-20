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
#include <boost/circular_buffer.hpp>
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

#include "affw/Config.h"
#include "affw/learner/DummyLearner.h"
#include "affw/learner/LWPRLearner.h"
#include "affw/learner/ModelLearner.h"
#include "affw/affw_common.h"
#include "affw/mapping/KTermStateTarget2ActionCompMapper.h"

typedef float AFFW_FLOAT;

// publishers
ros::Publisher targetRequest_pub;
ros::Publisher processing_time_pub;

// states
boost::circular_buffer<affw_msgs::State>* latestStates = NULL;
boost::mutex m_mutex;

affw::ModelLearner* learner = NULL;
affw::Config config;

// configs
ros::Duration timeStateDelayOffset(0.1);
ros::Duration timeAction2StateOffset(0.0);
bool updateModel = true;
int dim = 0;
int nFrames = 1;
std::string learner_type = "";

void fillStateVector(const std::vector<affw_msgs::State>& curStates, std::vector<AFFW_FLOAT>& vector)
{
	vector.reserve(nFrames * dim);
	std::vector<affw_msgs::State>::const_iterator it = curStates.end();
	for(int i=0;i<nFrames && i<curStates.size();i++)
	{
		it--;
		if(it < curStates.begin())
		{
			for(int j=0;j<dim;j++)
				vector.push_back(0);
		} else
		{
			vector.insert(vector.end(), it->vel.begin(), it->vel.end());
		}
	}
}

void getStateAtTime(const ros::Time& stamp, const std::vector<affw_msgs::State>& curStates, std::vector<AFFW_FLOAT>& intpState)
{
	if(curStates.size() < 2)
	{
		return;
	}

	typedef std::vector<affw_msgs::State>::const_iterator it_type;
	it_type postState = curStates.end();
	for(it_type it = curStates.begin(); it != curStates.end(); it++)
	{
		if(stamp <= it->header.stamp)
		{
			postState = it;
			break;
		}
	}
	if(postState == curStates.end())
	{
		ros::Duration diff = stamp - curStates.begin()->header.stamp;
		ROS_WARN("timestamp too recent. (%f)", diff.toSec());
	} else if(postState == curStates.begin())
	{
		ros::Duration diff = curStates.begin()->header.stamp - stamp;
		ROS_WARN("timestamp too old. (%f)", diff.toSec());
	}
	it_type preState = postState-1;

	ros::Duration state_dt = postState->header.stamp - preState->header.stamp;
	ros::Duration intp_dt = stamp - preState->header.stamp;
	for(int i=0;i<preState->vel.size();i++)
	{
		AFFW_FLOAT acc = 0;
		if(fabsf(state_dt.toSec()) > 0.001)
		{
			acc = (postState->vel[i] - preState->vel[i]) / state_dt.toSec();
		}
		intpState[i] = preState->vel[i] + acc * intp_dt.toSec();
	}
}

bool actionRequest(affw_msgs::ActionRequest::Request &req,
		affw_msgs::ActionRequest::Response &res) {

	// get curStates
	boost::mutex::scoped_lock lock(m_mutex);
	std::vector<affw_msgs::State> curStates;
	curStates.insert(curStates.end(), latestStates->begin(), latestStates->end());
	lock.unlock();

	if(curStates.empty())
	{
		return false;
	}

	ros::Duration curTimeAction2StateOffset = req.header.stamp - (curStates.end()-1)->header.stamp;
	if(curTimeAction2StateOffset > timeAction2StateOffset)
	{
		timeAction2StateOffset = curTimeAction2StateOffset;
		ROS_INFO_DELAYED_THROTTLE(1, "action to state timeOffset changed: %f", timeAction2StateOffset.toSec());
	}

	// TODO multiple frames not possible this way
	std::vector<AFFW_FLOAT> state;
	state.reserve(nFrames * dim);
//	fillStateVector(curStates, state);
	ros::Time stamp = req.header.stamp;
	ros::Time actionStamp = stamp;
	for(int i=0;i<nFrames;i++)
	{
		std::vector<AFFW_FLOAT> frameState(dim);
		actionStamp -= timeAction2StateOffset;
		getStateAtTime(actionStamp, curStates, frameState);
		state.insert(state.end(), frameState.begin(), frameState.end());
	}

	affw_msgs::TargetRequest tr;
	tr.header.stamp = stamp + timeStateDelayOffset;
	tr.timeAction2StateOffset = timeAction2StateOffset;
	tr.timeStateDelayOffset = timeStateDelayOffset;
	tr.state = state;
	tr.target = req.setPoint;
	tr.action = req.setPoint;

	if(state.size() == nFrames * dim)
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
	} else {
		std::cout << state.size() << std::endl;
	}

	res.outVel.insert(res.outVel.end(), tr.action.begin(), tr.action.end());
	assert(res.outVel.size() == req.setPoint.size());

	return true;
}

void feedbackStateCallback(const affw_msgs::State::ConstPtr& state) {

	if(learner == NULL)
	{
		dim = state->vel.size();
		config.setInt("actionDim", dim);
		config.setInt("nFrames", nFrames);
		latestStates = new boost::circular_buffer<affw_msgs::State>(nFrames*10);

		// TODO create somewhere else and delete
		affw::DataMapper* dm = new affw::KTermStateTarget2ActionCompMapper(config);
		if(learner_type == "lwpr") {
			learner = new affw::LWPR_Learner(config, dm);
			std::cout << "LWPR learner created" << std::endl;
		} else
		{
			learner = new affw::DummyLearner(config, dm);
			std::cout << "dummy learner created" << std::endl;
		}
	}

	boost::mutex::scoped_lock lock(m_mutex);
	latestStates->push_back(*state);
	lock.unlock();
}

void syncCallback(const affw_msgs::State::ConstPtr& state,
		const affw_msgs::TargetRequest::ConstPtr& target)
{
	affw::Vector
		l_state,
		l_target(dim),
		l_action(dim),
		l_action_compensation(dim),
		l_next_state(dim);

	l_state.insert(l_state.end(), target->state.begin(), target->state.end());

	for(int i=0;i<dim;i++)
	{
		l_target[i] = target->target[i];
		l_action[i] = target->action[i];
		l_action_compensation[i] = target->actionComp[i];
		l_next_state[i] = state->vel[i];
	}

	std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
	if(updateModel)	learner->addData(l_state,l_target,l_action,l_action_compensation,l_next_state);
	std::chrono::high_resolution_clock::time_point stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( stop - start ).count();
	affw_msgs::ProcTime proc;
	proc.stamp = target->header.stamp;
	proc.type = 2;
	proc.procTime = duration;
	processing_time_pub.publish(proc);
}

void saveModelCallback(const std_msgs::String::ConstPtr& location) {
	if(learner != NULL)
	{
		learner->write(location->data);
	}
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
	std::string configName = dataFolder + "/affw_" + learner_type + ".cfg";

	// create dataFolder, if it does not exist
	boost::filesystem::create_directories(dataFolder);

	if(!reset)
	{
		// read config, if it exists
		if(boost::filesystem::exists(configName))
		{
			config.read(configName.c_str());
			ROS_INFO_STREAM("Read affw config: " << configName);
		}
	}
	config.setString("dataFolder", dataFolder);
	config.setBool("reset", reset);
	updateModel = config.getBool("updateModel", updateModel);
	timeStateDelayOffset = ros::Duration(config.getDouble("timeOffset", timeStateDelayOffset.toSec()));
	nFrames = config.getInt("nFrames", nFrames);

	// debugging:
	std::cout << "Current config:" << std::endl;
	config.print();

	// set up state synchronization
	message_filters::Subscriber<affw_msgs::TargetRequest> targetRequest_sub(n, "/affw_ctrl/target_request", 10);
	message_filters::Subscriber<affw_msgs::State> state_sub(n, "/affw_ctrl/state", 10);
	typedef message_filters::sync_policies::ApproximateTime<affw_msgs::State, affw_msgs::TargetRequest> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), state_sub, targetRequest_sub);
	sync.registerCallback(boost::bind(&syncCallback, _1, _2));

	// set up topics
	targetRequest_pub = n.advertise<affw_msgs::TargetRequest>("/affw_ctrl/target_request", 10);
	processing_time_pub = n.advertise<affw_msgs::ProcTime>("/affw_ctrl/proc_time", 10);
	ros::ServiceServer service = n.advertiseService("/affw_ctrl/action", actionRequest);
	ros::Subscriber sub_fdbk_vel = n.subscribe("/affw_ctrl/state", 10, feedbackStateCallback);
	ros::Subscriber sub_save = n.subscribe("/affw_ctrl/save_model", 1, saveModelCallback);
	ros::Subscriber sub_updateModel = n.subscribe("/affw_ctrl/update_model", 1, updateModelCallback);

	// gogogo
	ros::MultiThreadedSpinner spinner;
	spinner.spin();

	// save data
	config.setBool("updateModel", updateModel);
	if(learner != NULL) learner->write(dataFolder);
	if(config.write(configName.c_str()))
	{
		std::cout << "Wrote affw config to " << configName << std::endl;
	} else {
		std::cout << "Could not write affw config to " << configName << std::endl;
	}

	if(latestStates != NULL) delete latestStates;
	if(learner != NULL)	delete learner;

	return 0;
}

