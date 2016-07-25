/*
 * RLSLearner.h
 *
 *  Created on: Jul 14, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_LIB_AFFW_SRC_LEARNER_RLSLEARNER_H_
#define AFFW_AFFW_CTRL_LIB_AFFW_SRC_LEARNER_RLSLEARNER_H_

#include "ModelLearner.h"

#include <otl_rls.h>

namespace affw {

class RLSLearner: public ModelLearner {
public:
	RLSLearner(Config& config);
	virtual ~RLSLearner();
	void addData(	const Vector& state,
					const Vector& target,
					const Vector& action,
					const Vector& actionComp,
					const Vector& nextState,
					const Vector& y);
	Vector getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug);
	void read(const std::string& folder);
	void write(const std::string& folder);
	static std::string name() { return "rls"; }
	std::string getName();
private:
	std::vector<OTL::RLS*> model;
	bool modelPerDim;
};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_LIB_AFFW_SRC_LEARNER_RLSLEARNER_H_ */
