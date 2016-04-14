/*
 * ModelLearner.h
 *
 *  Created on: Mar 24, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_CTRL_SRC_MODELLEARNER_H_
#define AFFW_CTRL_SRC_MODELLEARNER_H_

#include <vector>

namespace affw {

typedef std::vector<double> Vector;

class ModelLearner {
public:
	ModelLearner();
	virtual ~ModelLearner();
	virtual void addData(const Vector& state, const Vector& target, const Vector& action, const Vector& actionComp, const Vector& nextState) = 0;
	virtual Vector getActionCompensation(const Vector& state, const Vector& target) = 0;
private:
};

} /* namespace affw */

#endif /* AFFW_CTRL_SRC_MODELLEARNER_H_ */
