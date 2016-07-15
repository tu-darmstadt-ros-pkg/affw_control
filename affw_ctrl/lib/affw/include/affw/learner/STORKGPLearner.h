/*
 * STORKGPLearner.h
 *
 *  Created on: Jun 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_LIB_SRC_LEARNER_STORKGPLEARNER_H_
#define AFFW_AFFW_CTRL_LIB_SRC_LEARNER_STORKGPLEARNER_H_

#include "ModelLearner.h"
#include "affw/otl/STORKGP_async.h"

namespace affw {

class STORKGPLearner: public ModelLearner {
public:
	STORKGPLearner(Config& config);
	virtual ~STORKGPLearner();
	void addData(	const Vector& state,
					const Vector& target,
					const Vector& action,
					const Vector& actionComp,
					const Vector& nextState,
					const Vector& y);
	Vector getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug);
	void read(const std::string& folder);
	void write(const std::string& folder);
	static std::string name() { return "storkgp"; }
	std::string getName();

private:
	affw::STORKGPasync model;
};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_LIB_SRC_LEARNER_STORKGPLEARNER_H_ */
