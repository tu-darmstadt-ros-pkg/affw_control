/*
 * WrapperLearner.h
 *
 *  Created on: Jun 15, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_LIB_SRC_LEARNER_WRAPPERLEARNER_H_
#define AFFW_AFFW_CTRL_LIB_SRC_LEARNER_WRAPPERLEARNER_H_

#include "ModelLearner.h"

namespace affw {

class WrapperLearner : public ModelLearner {
public:
	WrapperLearner(Config& config);
	virtual ~WrapperLearner();
	void addData(	const Vector& state,
					const Vector& target,
					const Vector& action,
					const Vector& actionComp,
					const Vector& nextState,
					const Vector& y);
	Vector getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug);
	void read(const std::string& folder);
	void write(const std::string& folder);
	static std::string name() { return "wrapper"; }
	std::string getName();

private:
	std::vector<ModelLearner*> modelLearners;
};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_LIB_SRC_LEARNER_WRAPPERLEARNER_H_ */
