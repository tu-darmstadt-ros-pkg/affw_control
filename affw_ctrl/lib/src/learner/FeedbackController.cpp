/*
 * FeedbackController.cpp
 *
 *  Created on: Mar 26, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/learner/FeedbackController.h"

namespace affw {

FeedbackController::FeedbackController(Config& config, DataMapper* dataMapper)
	: ModelLearner(config, dataMapper)
{
	this->dataMapper = dataMapper;
}

FeedbackController::~FeedbackController() {
}

void FeedbackController::addData(
		const Vector& state,
		const Vector& target,
		const Vector& action,
		const Vector& actionComp,
		const Vector& nextState,
			  Vector& y)
{
	Vector x;
	dataMapper->getInput(state, target, x);
	dataMapper->getOutput(state, target, action, actionComp, nextState, y);
	this->actionComp = y;

//	if(this->actionComp.size() != action.size())
//	{
//		this->actionComp.resize(action.size(), 0);
//	}
//
//	for(int i=0;i<action.size();i++)
//	{
//		this->actionComp[i] = 1 * (nextState[i] - state[i]);
//	}
//
//	y = this->actionComp;
}

Vector FeedbackController::getActionCompensation(const Vector& state, const Vector& target, Vector& learnerDebug)
{
	if(actionComp.size() != target.size())
	{
		actionComp.resize(target.size(), 0);
	}
	return actionComp;
}

void FeedbackController::read(const std::string& folder)
{
}

void FeedbackController::write(const std::string& folder)
{
}

} /* namespace affw */
