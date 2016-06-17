/*
 * FANNLearner.h
 *
 *  Created on: May 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_SRC_LEARNER_FANNLEARNER_H_
#define AFFW_AFFW_CTRL_SRC_LEARNER_FANNLEARNER_H_

#include "ModelLearner.h"
#include <string>
#include <fann.h>

namespace affw {

class FANNLearner: public ModelLearner {
public:
	FANNLearner(Config& config);
	virtual ~FANNLearner();
	void addData(	const Vector& state,
					const Vector& target,
					const Vector& action,
					const Vector& actionComp,
					const Vector& nextState,
					const Vector& y);
	Vector getActionCompensation(const Vector& state, const Vector& target, Vector& learnerDebug);
	void read(const std::string& folder);
	void write(const std::string& folder);

private:
	struct fann *ann;
};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_SRC_LEARNER_FANNLEARNER_H_ */
