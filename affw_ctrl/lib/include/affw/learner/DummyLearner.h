/*
 * DummyLearner.h
 *
 *  Created on: Mar 26, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_CTRL_SRC_DUMMYLEARNER_H_
#define AFFW_CTRL_SRC_DUMMYLEARNER_H_

#include "affw/learner/ModelLearner.h"

namespace affw {

class DummyLearner: public ModelLearner {
public:
	DummyLearner(Config& config, DataMapper* dataMapper);
	virtual ~DummyLearner();
	void addData(const Vector& state, const Vector& target, const Vector& action, const Vector& actionComp, const Vector& nextState);
	Vector getActionCompensation(const Vector& state, const Vector& target);
	void write(const std::string& folder);
};

} /* namespace affw */

#endif /* AFFW_CTRL_SRC_DUMMYLEARNER_H_ */
