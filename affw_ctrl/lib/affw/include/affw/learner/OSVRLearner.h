/*
 * OSVRLearner.h
 *
 *  Created on: Jul 20, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_LIB_AFFW_SRC_LEARNER_OSVRLEARNER_H_
#define AFFW_AFFW_CTRL_LIB_AFFW_SRC_LEARNER_OSVRLEARNER_H_

#include "ModelLearner.h"
#include "affw/osvr/OnlineSVR.h"

namespace affw {

class OSVRLearner : public ModelLearner{
public:
	OSVRLearner(Config& config);
	virtual ~OSVRLearner();
	void addData(	const Vector& state,
					const Vector& target,
					const Vector& action,
					const Vector& actionComp,
					const Vector& nextState,
					const Vector& y);
	Vector getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug);
	void read(const std::string& folder);
	void write(const std::string& folder);
	static std::string name() { return "osvr"; }
	std::string getName();

private:
	std::vector<onlinesvr::OnlineSVR*> models;
};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_LIB_AFFW_SRC_LEARNER_OSVRLEARNER_H_ */
