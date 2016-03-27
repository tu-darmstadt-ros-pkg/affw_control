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
	virtual void addData(Vector state, Vector target, Vector action, Vector actionComp, Vector nextState) = 0;
	virtual Vector getActionCompensation(Vector state, Vector target) = 0;
private:
};

} /* namespace affw */

#endif /* AFFW_CTRL_SRC_MODELLEARNER_H_ */
