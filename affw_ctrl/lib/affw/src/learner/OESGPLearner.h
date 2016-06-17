/*
 * OTLLearner.h
 *
 *  Created on: Jun 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_LIB_SRC_LEARNER_OTLLEARNER_H_
#define AFFW_AFFW_CTRL_LIB_SRC_LEARNER_OTLLEARNER_H_

#include "ModelLearner.h"
#include "otl.h"
#include "otl_oesgp.h"

namespace affw {

class OESGPLearner: public ModelLearner {
public:
	OESGPLearner(Config& config);
	virtual ~OESGPLearner();
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
	OTL::OESGP model;

};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_LIB_SRC_LEARNER_OTLLEARNER_H_ */
