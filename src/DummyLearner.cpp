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

void DummyLearner::addData(Vector state, Vector target, Vector action, Vector actionComp, Vector nextState)
{
}

Vector DummyLearner::getActionCompensation(Vector state, Vector target)
{
	Vector v(target.size());
	return v;
}

} /* namespace affw */
