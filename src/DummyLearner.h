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
	void addData(Vector state, Vector target, Vector action, Vector actionComp, Vector nextState);
	Vector getActionCompensation(Vector state, Vector target);
};

} /* namespace affw */

#endif /* AFFW_CTRL_SRC_DUMMYLEARNER_H_ */
