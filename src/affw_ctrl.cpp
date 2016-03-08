/*
 * affw_ctrl.cpp
 *
 *  Created on: 24.02.2016
 *      Author: NicolaiO
 */

#include "ros/ros.h"
#include "affw_ctrl/ActionRequest.h"
#include "affw_ctrl/State.h"

#include <iostream>
#include <fstream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

typedef struct DataSet_ {
	double t;
	std::vector<float> lvel;
	std::vector<float> gvel;
	std::vector<float> gpos;
} DataSet;

typedef struct SetPoint_ {
	double t;
	std::vector<float> vel;
} SetPoint;

ros::Time timeOffset;
std::vector<SetPoint_> set_vels;
std::vector<DataSet> state_vels;

double getTime(ros::Time t) {
	ros::Duration d = t - timeOffset;
	return d.toSec();
}

bool actionRequest(affw_ctrl::ActionRequest::Request &req,
		affw_ctrl::ActionRequest::Response &res) {
	res.outVel = req.setPoint;
	SetPoint ds;
	ds.t = getTime(req.t);
	ds.vel = req.setPoint;
	set_vels.push_back(ds);

	return true;
}

void feedbackStateCallback(const affw_ctrl::State::ConstPtr& state) {
	DataSet ds;
	ds.t = getTime(state->t);
	ds.lvel = state->local_vel;
	ds.gvel = state->global_vel;
	ds.gpos = state->global_pos;

	state_vels.push_back(ds);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "affw_ctrl");
	ros::NodeHandle n;

	while (ros::Time::now().isZero())
		;
	timeOffset = ros::Time::now();

	ros::Subscriber sub_fdbk_vel = n.subscribe("/affw_ctrl/state", 1,
			feedbackStateCallback);

	ros::ServiceServer service = n.advertiseService("/affw_ctrl/action",
			actionRequest);

	ros::spin();

	std::vector<SetPoint>::iterator itSet = set_vels.begin();
	std::vector<DataSet>::iterator itState = state_vels.begin();

	int numDataSets = 0;
	if (itSet < set_vels.end()) {
		std::ofstream myfile;
		myfile.open("/tmp/traj_out.csv");

		// seek forward until first set point data
		while (itState < state_vels.end() && itState->t < itSet->t)
			itState++;

		while (itSet < set_vels.end()) {
			std::vector<float> setVel = itSet->vel;
			itSet++;
			while (itState < state_vels.end() && itState->t < itSet->t) {
				itState++;
				myfile << itState->t;
				for (int i = 0; i < setVel.size(); i++)
					myfile << " " << setVel[i];
				for (int i = 0; i < itState->lvel.size(); i++)
					myfile << " " << itState->lvel[i];
				for (int i = 0; i < itState->gvel.size(); i++)
					myfile << " " << itState->gvel[i];
				for (int i = 0; i < itState->gpos.size(); i++)
					myfile << " " << itState->gpos[i];
				myfile << std::endl;
				numDataSets++;
			}
		}
		myfile.close();
	}

	std::cout << "Exported " << numDataSets << " data sets" << std::endl;

	return 0;
}

