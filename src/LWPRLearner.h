/*
 * LWPRLearner.h
 *
 *  Created on: Mar 15, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_CTRL_SRC_LWPRLEARNER_H_
#define AFFW_CTRL_SRC_LWPRLEARNER_H_

#include <lwpr.hh>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "ModelLearner.h"

namespace affw {

class LWPR_Learner : public ModelLearner {
public:
	LWPR_Learner(int nFrames, int actionDim);
	virtual ~LWPR_Learner();
	void addData(const Vector& state, const Vector& target, const Vector& action, const Vector& actionComp, const Vector& nextState);
	Vector getActionCompensation(const Vector& state, const Vector& target);
private:
	LWPR_Object model;
	double cutoff;
	std::vector<double> update(std::vector<double>, std::vector<double>);
	std::vector<double> predict(std::vector<double>);
};

}

#endif /* AFFW_CTRL_SRC_LWPRLEARNER_H_ */
