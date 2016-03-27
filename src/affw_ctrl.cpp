/*
 * affw_ctrl.cpp
 *
 *  Created on: 24.02.2016
 *      Author: NicolaiO
 */

#include <affw_ctrl/ActionRequest.h>
#include <affw_ctrl/ActionRequestRequest.h>
#include <affw_ctrl/State.h>
#include <affw_ctrl/TargetRequest.h>
#include <boost/bind/arg.hpp>
#include <boost/bind/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <lwpr.hh>
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
#include <stddef.h>
#include <iostream>
#include <queue>
#include <vector>

#include "LWPRLearner.h"
#include "ModelLearner.h"

ros::Publisher targetRequest_pub;

ros::Duration timeOffset(0.1);
std::vector<double> curState;

int nFrames = 4;
affw::ModelLearner* learner = NULL;

bool actionRequest(affw_ctrl::ActionRequest::Request &req,
		affw_ctrl::ActionRequest::Response &res) {

	affw_ctrl::TargetRequest tr;
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
		affw::Vector action = learner->getActionCompensation(s, t);
		tr.actionComp.reserve(action.size());
		for(int i=0;i<action.size(); i++)
		{
			tr.actionComp.push_back(action[i]);
			tr.action[i] += action[i];
		}
		targetRequest_pub.publish(tr);
		ros::spinOnce();
	}

	res.outVel.insert(res.outVel.end(), tr.action.begin(), tr.action.end());
	assert(res.outVel.size() == req.setPoint.size());

	return true;
}

void feedbackStateCallback(const affw_ctrl::State::ConstPtr& state) {
	int dim = state->vel.size();

	if(learner == NULL)
		learner = new affw::LWPR_Learner(nFrames,dim);

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

void syncCallback(const affw_ctrl::State::ConstPtr& state,
		const affw_ctrl::TargetRequest::ConstPtr& target)
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

	learner->addData(s,t,a,ac,ns);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "affw_ctrl");
	ros::NodeHandle n;

	targetRequest_pub = n.advertise<affw_ctrl::TargetRequest>("/affw_ctrl/target_request", 1);
	message_filters::Subscriber<affw_ctrl::TargetRequest> targetRequest_sub(n, "/affw_ctrl/target_request", 1);

	message_filters::Subscriber<affw_ctrl::State> state_sub(n, "/affw_ctrl/state", 1);

	typedef message_filters::sync_policies::ApproximateTime<affw_ctrl::State, affw_ctrl::TargetRequest> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), state_sub, targetRequest_sub);
	sync.registerCallback(boost::bind(&syncCallback, _1, _2));

	ros::Subscriber sub_fdbk_vel = n.subscribe("/affw_ctrl/state", 1,
			feedbackStateCallback);

	ros::ServiceServer service = n.advertiseService("/affw_ctrl/action",
			actionRequest);

	ros::spin();

	return 0;
}

