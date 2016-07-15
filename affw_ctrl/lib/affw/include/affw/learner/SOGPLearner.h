/*
 * STORKGPLearner.h
 *
 *  Created on: Jun 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_LIB_SRC_LEARNER_SOGPLEARNER_H_
#define AFFW_AFFW_CTRL_LIB_SRC_LEARNER_SOGPLEARNER_H_

#include "ModelLearner.h"
#include "otl.h"
#include "otl_sogp.h"
#include "otl_kernel_recursive_gaussian.h"

namespace affw {

class SOGPLearner: public ModelLearner {
public:
	SOGPLearner(Config& config);
	virtual ~SOGPLearner();
	void addData(	const Vector& state,
					const Vector& target,
					const Vector& action,
					const Vector& actionComp,
					const Vector& nextState,
					const Vector& y);
	Vector getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug);
	void read(const std::string& folder);
	void write(const std::string& folder);
	static std::string name() { return "sogp"; }
	std::string getName();

private:
    OTL::RecursiveGaussianKernel kernel;
	OTL::SOGP model;

};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_LIB_SRC_LEARNER_SOGPLEARNER_H_ */
