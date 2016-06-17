/*
 * DummyLearner.cpp
 *
 *  Created on: Mar 26, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "DummyLearner.h"

namespace affw {

DummyLearner::DummyLearner(Config& config)
	: ModelLearner(config)
{

}

DummyLearner::~DummyLearner() {
}

void DummyLearner::addData(
		const Vector& state,
		const Vector& target,
		const Vector& action,
		const Vector& actionComp,
		const Vector& nextState,
		const Vector& y)
{
}

Vector DummyLearner::getActionCompensation(const Vector& state, const Vector& target, Vector& learnerDebug)
{
	Vector v(target.size());
	return v;
}

void DummyLearner::read(const std::string& folder)
{
}

void DummyLearner::write(const std::string& folder)
{
}

} /* namespace affw */
