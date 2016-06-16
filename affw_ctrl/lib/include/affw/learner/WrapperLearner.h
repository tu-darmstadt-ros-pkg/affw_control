/*
 * WrapperLearner.h
 *
 *  Created on: Jun 15, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_LIB_SRC_LEARNER_WRAPPERLEARNER_H_
#define AFFW_AFFW_CTRL_LIB_SRC_LEARNER_WRAPPERLEARNER_H_

#include "affw/learner/ModelLearner.h"

namespace affw {

class WrapperLearner : public ModelLearner {
public:
	WrapperLearner(Config& config, DataMapper* dataMapper);
	virtual ~WrapperLearner();
	void addData(	const Vector& state,
					const Vector& target,
					const Vector& action,
					const Vector& actionComp,
					const Vector& nextState,
						  Vector& y);
	Vector getActionCompensation(const Vector& state, const Vector& target, Vector& learnerDebug);
	void read(const std::string& folder);
	void write(const std::string& folder);

private:
	std::vector<ModelLearner*> modelLearners;
};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_LIB_SRC_LEARNER_WRAPPERLEARNER_H_ */
