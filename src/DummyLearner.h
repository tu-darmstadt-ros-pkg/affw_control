/*
 * DummyLearner.h
 *
 *  Created on: Mar 26, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_CTRL_SRC_DUMMYLEARNER_H_
#define AFFW_CTRL_SRC_DUMMYLEARNER_H_

#include "ModelLearner.h"

namespace affw {

class DummyLearner: public ModelLearner {
public:
	DummyLearner();
	virtual ~DummyLearner();
	void addData(const Vector& state, const Vector& target, const Vector& action, const Vector& actionComp, const Vector& nextState);
	Vector getActionCompensation(const Vector& state, const Vector& target);
};

} /* namespace affw */

#endif /* AFFW_CTRL_SRC_DUMMYLEARNER_H_ */
