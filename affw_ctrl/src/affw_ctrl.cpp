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
#include <signal.h>

#include <affw/affw.h>

typedef float AFFW_FLOAT;

// publishers
ros::Publisher targetRequest_pub;
ros::Publisher target4Sync_pub;
ros::Publisher processing_time_pub;

// states
boost::circular_buffer<affw_msgs::State>* latestStates = NULL;
boost::circular_buffer<ros::Duration> action2StateOffsets(20);
ros::Duration maxAction2StateOffset(0);
ros::Duration avgAction2StateOffset(-1);
boost::mutex m_mutex;
affw::Vector lastActionComp;

affw::ModelLearner* learner = NULL;
affw::DataMapper* dataMapper = NULL;
affw::Config config;

affw_msgs::ActionRequest::Request lastReq;

// configs
ros::Duration timeStateDelayOffset(0.1);
ros::Duration timeAction2StateOffset(-1.0);
ros::Duration frameDt(0.01);
bool updateModel = true;
int nFrames = 1;
bool useAcc = true;
bool interpState = false;
bool useLastActionComp = false;
std::string learner_type = "";
std::string configFolder;
std::string configName;

bool getStateAtTime(const ros::Time& stamp, const std::vector<affw_msgs::State>& curStates, std::vector<AFFW_FLOAT>& intpState)
{
	if(curStates.size() < 2)
	{
		return false;
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
		ros::Duration diff = stamp - (curStates.end()-1)->header.stamp;
		if(diff.toSec() > 0.5)
		{
			ROS_WARN("timestamp much too recent. (%f)", diff.toSec());
		}
		postState = curStates.end() - 1;
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

	return true;
}

bool actionRequest(affw_msgs::ActionRequest::Request &req,
		affw_msgs::ActionRequest::Response &res) {

	int dim = req.state.vel.size();
	int stateDim = (nFrames + (useAcc?1:0) + 1 + (useLastActionComp?1:0)) * dim + req.state.custom.size();

	if(learner == NULL)
	{
		config.setInt("actionDim", dim);
		config.setInt("stateDim", stateDim);

		dataMapper = new affw::KTermStateTarget2ActionCompMapper(config);
		if(learner_type == "lwpr") {
			learner = new affw::LWPR_Learner(config);
			ROS_INFO("LWPR learner created");
		} else if(learner_type == "fann")
		{
			learner = new affw::FANNLearner(config);
			ROS_INFO("FANN learner created");
		} else if(learner_type == "fdbk")
		{
			learner = new affw::FeedbackController(config);
			ROS_INFO("Feedback Controller created");
		} else if(learner_type == "wrap")
		{
			learner = new affw::WrapperLearner(config);
			ROS_INFO("Wrapper learner created");
		} else if(learner_type == "oesgp")
		{
			learner = new affw::OESGPLearner(config);
			ROS_INFO("OESGP learner created");
		} else
		{
			learner = new affw::DummyLearner(config);
			ROS_INFO("dummy learner created");
		}

		bool loadModel = config.getBool("reload_model", false);
		if(loadModel)
		{
			learner->read(configFolder);
		}
	}

	// get curStates
	std::vector<affw_msgs::State> curStates;
	boost::mutex::scoped_lock lock(m_mutex);
	curStates.insert(curStates.end(), latestStates->begin(), latestStates->end());
	lock.unlock();
	if(curStates.empty())
	{
		return false;
	}

	// prepare state for learner
	std::vector<AFFW_FLOAT> state;
	state.reserve(nFrames * dim);

	// get acc
	if(useAcc)
	{
		state.reserve((1+nFrames) * dim);
		if(lastReq.state.vel.empty()) {
			lastReq = req;
		}

		std::vector<AFFW_FLOAT> acc(req.state.vel.size());
		double dt = (req.state.header.stamp - lastReq.state.header.stamp).toSec();
		if(dt > 0)
		{
			for(int i=0;i<req.state.vel.size();i++)
				acc[i] = (req.state.vel[i] - lastReq.state.vel[i]) / dt;
		}
		lastReq = req;

		state.insert(state.end(), acc.begin(), acc.end());
	}

	// state to action offset calculation
	ros::Duration curTimeAction2StateOffset = req.state.header.stamp - (curStates.end()-1)->header.stamp;
	action2StateOffsets.push_back(curTimeAction2StateOffset);
	// avg offset
	ros::Duration sum;
	std::for_each(action2StateOffsets.begin(), action2StateOffsets.end(),
	    [&](const ros::Duration& dur) {sum += dur;});
	double curAvgTimeAction2StateOffset = sum.toSec() / action2StateOffsets.size();
	if(!avgAction2StateOffset.toSec() < 0 || fabsf(curAvgTimeAction2StateOffset - timeAction2StateOffset.toSec()) > 0.01)
	{
		avgAction2StateOffset = ros::Duration(curAvgTimeAction2StateOffset);
		ROS_INFO_DELAYED_THROTTLE(1, "action to state timeOffset changed: %f", curAvgTimeAction2StateOffset);
	}

	// max offset
	if(curTimeAction2StateOffset > maxAction2StateOffset)
	{
		maxAction2StateOffset = curTimeAction2StateOffset;
		ROS_INFO_DELAYED_THROTTLE(1, "max action to state timeOffset changed: %f (avg=%f)"
				, maxAction2StateOffset.toSec(), avgAction2StateOffset.toSec());
	}

	if(interpState)
	{
		timeAction2StateOffset = avgAction2StateOffset;
	} else {
		timeAction2StateOffset = maxAction2StateOffset;
	}

	ros::Time stamp = req.state.header.stamp;
	ros::Time actionStamp = stamp - timeAction2StateOffset;
	bool validState = true;
	for(int i=0;i<nFrames;i++)
	{
		std::vector<AFFW_FLOAT> frameState(dim);
		validState = validState && getStateAtTime(actionStamp, curStates, frameState);
		state.insert(state.end(), frameState.begin(), frameState.end());
		actionStamp -= frameDt;
	}
	if(!validState)
	{
		return false;
	}

	if(lastActionComp.empty())
		lastActionComp.resize(dim, 0);

	state.insert(state.end(), req.state.custom.begin(), req.state.custom.end());
	state.insert(state.end(), req.state.vel.begin(), req.state.vel.end());
	if(useLastActionComp)
		state.insert(state.end(), lastActionComp.begin(), lastActionComp.end());

	affw_msgs::TargetRequest tr;
	tr.header.stamp = stamp + timeStateDelayOffset;
	tr.timeAction2StateOffset = timeAction2StateOffset;
	tr.timeStateDelayOffset = timeStateDelayOffset;
	tr.state = state;
	tr.target = req.state.vel;
	tr.action = req.state.vel;

	affw::Vector s(tr.state.begin(), tr.state.end());
	affw::Vector t(tr.target.begin(), tr.target.end());
	affw::Vector learnerDebug;

	// get action compensation
	std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
	affw::Vector actionComp = learner->getActionCompensation(s, t, learnerDebug);
	std::chrono::high_resolution_clock::time_point stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>( stop - start ).count();

	lastActionComp = actionComp;

	// export processing time
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
	tr.learnerDebug.reserve(learnerDebug.size());
	for(int i=0;i<learnerDebug.size();i++)
		tr.learnerDebug.push_back(learnerDebug[i]);
	target4Sync_pub.publish(tr);
	ros::spinOnce();

	res.outVel.insert(res.outVel.end(), tr.action.begin(), tr.action.end());
	assert(res.outVel.size() == req.state.vel.size());

	return true;
}

void feedbackStateCallback(const affw_msgs::State::ConstPtr& state) {
	boost::mutex::scoped_lock lock(m_mutex);
	latestStates->push_back(*state);
	lock.unlock();
}

void syncCallback(const affw_msgs::State::ConstPtr& state,
		const affw_msgs::TargetRequest::ConstPtr& target)
{
	int dim = target->action.size();
	affw::Vector
		l_state,
		l_target(dim),
		l_action(dim),
		l_action_compensation(dim),
		l_next_state(dim),
		l_y(dim);

	l_state.insert(l_state.end(), target->state.begin(), target->state.end());

	for(int i=0;i<dim;i++)
	{
		l_target[i] = target->target[i];
		l_action[i] = target->action[i];
		l_action_compensation[i] = target->actionComp[i];
		l_next_state[i] = state->vel[i];
	}

	dataMapper->getOutput(l_state,l_target,l_action,l_action_compensation,l_next_state, l_y);

	std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
	// update model
	if(updateModel)	learner->addData(l_state,l_target,l_action,l_action_compensation,l_next_state, l_y);
	std::chrono::high_resolution_clock::time_point stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( stop - start ).count();

    // export full target request ( for debugging )
	affw_msgs::TargetRequest fullTargetReq = *target;
	fullTargetReq.nextState.insert(fullTargetReq.nextState.end(), l_next_state.begin(), l_next_state.end());
	fullTargetReq.nextActionComp.insert(fullTargetReq.nextActionComp.end(), l_y.begin(), l_y.end());
	targetRequest_pub.publish(fullTargetReq);

    // export processing time
	affw_msgs::ProcTime proc;
	proc.stamp = target->header.stamp;
	proc.type = 2;
	proc.procTime = duration;
	processing_time_pub.publish(proc);
}

void saveModelCallback(const std_msgs::String::ConstPtr& location) {
	ROS_INFO("save model");
	if(learner != NULL)
	{
		learner->write(location->data);
	}
}

void updateModelCallback(const std_msgs::Bool::ConstPtr& enabled)
{
	updateModel = enabled->data;
	if(updateModel)
	{
		ROS_INFO("Update model enabled");
	} else {
		ROS_INFO("Update model disabled");
	}
}

void mySigintHandler(int sig)
{
	// save data
	if(learner != NULL) learner->write(configFolder);
	if(config.write(configName.c_str()))
	{
		ROS_ERROR_STREAM("Could not write affw config to " << configName);
	}

	if(latestStates != NULL) delete latestStates;
	if(learner != NULL)	delete learner;

	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "affw_ctrl");
	ros::NodeHandle n;

	configFolder = boost::filesystem::current_path().string();
	ros::param::get("learner", learner_type);
	ros::param::get("configFolder", configFolder);
	std::string configName = configFolder + "/affw.cfg";

	// create dataFolder, if it does not exist
	boost::filesystem::create_directories(configFolder);

	// read config, if it exists
	if(boost::filesystem::exists(configName))
	{
		config.read(configName.c_str());
		ROS_DEBUG_STREAM("Read affw config: " << configName);
	}
	updateModel = config.getBool("updateModel", updateModel);
	timeStateDelayOffset = ros::Duration(config.getDouble("timeOffset", timeStateDelayOffset.toSec()));
	nFrames = config.getInt("nFrames", nFrames);
	latestStates = new boost::circular_buffer<affw_msgs::State>(std::max(1,nFrames)*20);
	useAcc = config.getBool("useAcc", useAcc);
	interpState = config.getBool("interpState", interpState);

	// debugging:
	ROS_INFO_STREAM("Current config:" << std::endl << config);

	// use unreliable connections
	ros::TransportHints th;
	th.unreliable();

	// set up state synchronization
	message_filters::Subscriber<affw_msgs::TargetRequest> targetRequest_sub(n, "/affw_ctrl/target4Sync", 10, th);
	message_filters::Subscriber<affw_msgs::State> state_sub(n, "/affw_ctrl/state", 10, th);
	typedef message_filters::sync_policies::ApproximateTime<affw_msgs::State, affw_msgs::TargetRequest> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(500), state_sub, targetRequest_sub);
	sync.registerCallback(boost::bind(&syncCallback, _1, _2));

	// set up topics
	targetRequest_pub = n.advertise<affw_msgs::TargetRequest>("/affw_ctrl/target_request", 10);
	target4Sync_pub = n.advertise<affw_msgs::TargetRequest>("/affw_ctrl/target4Sync", 10);
	processing_time_pub = n.advertise<affw_msgs::ProcTime>("/affw_ctrl/proc_time", 10);
	ros::ServiceServer service = n.advertiseService("/affw_ctrl/action", actionRequest);
	ros::Subscriber sub_fdbk_vel = n.subscribe("/affw_ctrl/state", 1, feedbackStateCallback, th);
	ros::Subscriber sub_save = n.subscribe("/affw_ctrl/save_model", 1, saveModelCallback);
	ros::Subscriber sub_updateModel = n.subscribe("/affw_ctrl/update_model", 10, updateModelCallback);


	signal(SIGINT, mySigintHandler);
	ros::MultiThreadedSpinner spinner;
	spinner.spin();
	return 0;
}

