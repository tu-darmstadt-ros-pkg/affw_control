/*
 * DummyLearner.cpp
 *
 *  Created on: Mar 26, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "../../../affw_ctrl/src/learner/DummyLearner.h"

namespace affw {

DummyLearner::DummyLearner() {

}

DummyLearner::~DummyLearner() {
}

void DummyLearner::addData(const Vector& state, const Vector& target, const Vector& action, const Vector& actionComp, const Vector& nextState)
{
}

Vector DummyLearner::getActionCompensation(const Vector& state, const Vector& target)
{
	Vector v(target.size());
	return v;
}

void DummyLearner::write(const std::string& folder)
{
}

} /* namespace affw */
