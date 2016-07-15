/*
 * FeedbackController.cpp
 *
 *  Created on: Mar 26, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/learner/FeedbackController.h"

namespace affw {

FeedbackController::FeedbackController(Config& config)
	: ModelLearner(FeedbackController::name(), config)
{
	p = config.getDoubleVector("kterm_st2ac.k", p);
}

FeedbackController::~FeedbackController() {
}

void FeedbackController::addData(
		const Vector& state,
		const Vector& target,
		const Vector& action,
		const Vector& actionComp,
		const Vector& nextState,
		const Vector& y)
{
//	this->actionComp = y;
	if(actionComp.size() != target.size())
	{
		this->actionComp.resize(target.size(), 0);
	}
	for(int i=0;i<target.size();i++)
	{
		this->actionComp[i] = y[i] - actionComp[i];
	}
}

Vector FeedbackController::getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug)
{
	if(actionComp.size() != target.size())
	{
		actionComp.resize(target.size(), 0);
	}
	return actionComp;

//	Vector y(target.size());
//	for(int i=0;i<target.size();i++)
//	{
//		y[i] = p[i] * (target[i] - preState[i]);
//	}
//	return y;
}

void FeedbackController::read(const std::string& folder)
{
}

void FeedbackController::write(const std::string& folder)
{
}

} /* namespace affw */
