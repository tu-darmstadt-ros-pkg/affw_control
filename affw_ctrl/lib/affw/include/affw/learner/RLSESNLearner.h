/*
 * RLSESNLearner.h
 *
 *  Created on: Jul 14, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_LIB_AFFW_SRC_LEARNER_RLSESNLearner_H_
#define AFFW_AFFW_CTRL_LIB_AFFW_SRC_LEARNER_RLSESNLearner_H_

#include "ModelLearner.h"

#include "affw/otl/otl_rls_osn.h"

namespace affw {

class RLSESNLearner: public ModelLearner {
public:
	RLSESNLearner(Config& config);
	virtual ~RLSESNLearner();
	void addData(	const Vector& state,
					const Vector& target,
					const Vector& action,
					const Vector& actionComp,
					const Vector& nextState,
					const Vector& y);
	Vector getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug);
	void read(const std::string& folder);
	void write(const std::string& folder);
	static std::string name() { return "rlsesn"; }
	std::string getName();
private:
	affw::RLSESN model;
};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_LIB_AFFW_SRC_LEARNER_RLSESNLearner_H_ */
