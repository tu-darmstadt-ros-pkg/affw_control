/*
 * FeedbackController.cpp
 *
 *  Created on: Mar 26, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/learner/FeedbackController.h"

namespace affw {

FeedbackController::FeedbackController(Config& config)
	: ModelLearner(config)
{
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
	this->actionComp = y;
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
