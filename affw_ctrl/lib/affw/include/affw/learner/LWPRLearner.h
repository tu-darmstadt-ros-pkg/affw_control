/*
 * LWPRLearner.h
 *
 *  Created on: Mar 15, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_CTRL_SRC_LWPRLEARNER_H_
#define AFFW_CTRL_SRC_LWPRLEARNER_H_

#include "ModelLearner.h"
#include <string>
#include <vector>
#include <lwpr.hh>

namespace affw {

class LWPR_Learner : public ModelLearner {
public:
	LWPR_Learner(Config& config);
	virtual ~LWPR_Learner();
	void addData(	const Vector& state,
					const Vector& target,
					const Vector& action,
					const Vector& actionComp,
					const Vector& nextState,
					const Vector& y);
	Vector getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug);
	void read(const std::string& folder);
	void write(const std::string& folder);
	static std::string name() { return "lwpr"; }
	std::string getName();
private:
	void updateModel();

	LWPR_Object* model;
	double cutoff;
};

}

#endif /* AFFW_CTRL_SRC_LWPRLEARNER_H_ */
