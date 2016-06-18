/*
 * WrapperLearner.cpp
 *
 *  Created on: Jun 15, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/learner/WrapperLearner.h"
#include "affw/learner/LWPRLearner.h"

namespace affw {

WrapperLearner::WrapperLearner(Config& config)
	: ModelLearner(config)
{
	int actionDim = config.getInt("actionDim", 1);
	std::vector<double> upperOutputBounds(actionDim, 0);
	upperOutputBounds = config.getDoubleVector("upperOutputBounds", upperOutputBounds);

	std::vector<double> c_k(1,0);
	c_k = config.getDoubleVector("kterm_st2ac.k", c_k);

	for(int i=0;i<actionDim;i++)
	{
		Config cfg = config;
		cfg.setInt("actionDim", 1);
		std::vector<double> uob(1);
		uob[0] = upperOutputBounds[i];
		cfg.setDoubleVector("upperOutputBounds", uob);
		std::vector<double> k(1);
		k[0] = c_k[i];
		cfg.setDoubleVector("kterm_st2ac.k", k);
		ModelLearner* ml = new LWPR_Learner(cfg);
		modelLearners.push_back(ml);
	}
}

WrapperLearner::~WrapperLearner() {
}

void WrapperLearner::addData(
		const Vector& state,
		const Vector& target,
		const Vector& action,
		const Vector& actionComp,
		const Vector& nextState,
		const Vector& y)
{
	for(int i=0;i<action.size();i++)
	{
		Vector l_target(1);
		Vector l_action(1);
		Vector l_actionComp(1);
		Vector l_nextState(1);
		Vector l_y(1);
		l_target[0] = target[i];
		l_action[0] = action[i];
		l_actionComp[0] = actionComp[i];
		l_nextState[0] = nextState[i];
		l_y[0] = y[i];

		modelLearners[i]->addData(state, l_target, l_action, l_actionComp, l_nextState, l_y);
	}
}

Vector WrapperLearner::getActionCompensation(const Vector& state, const Vector& target, Vector& learnerDebug)
{
	Vector v(target.size());

	for(int i=0;i<target.size();i++)
	{
		Vector l_target(1);
		Vector l_ld;
		l_target[0] = target[i];
		Vector l_v = modelLearners[i]->getActionCompensation(state, l_target, l_ld);
		v[i] = l_v[0];
		if(learnerDebug.empty())
		{
			learnerDebug.resize(l_ld.size()*3);
		}
		for(int j=0;j<l_ld.size();j++)
		{
			learnerDebug[i*l_ld.size()+j] = l_ld[j];
		}
	}

	return v;
}

void WrapperLearner::read(const std::string& folder)
{
	for(int i=0;i<modelLearners.size();i++)
	{
		modelLearners[i]->read(folder + "/" + std::to_string(i));
	}
}

void WrapperLearner::write(const std::string& folder)
{
	for(int i=0;i<modelLearners.size();i++)
	{
		modelLearners[i]->write(folder + "/" + std::to_string(i));
	}
}

} /* namespace affw */
