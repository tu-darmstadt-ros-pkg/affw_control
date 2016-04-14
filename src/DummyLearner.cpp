/*
 * DummyLearner.cpp
 *
 *  Created on: Mar 26, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "DummyLearner.h"

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

} /* namespace affw */
