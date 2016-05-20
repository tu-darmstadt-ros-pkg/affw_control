/*
 * ModelLearner.h
 *
 *  Created on: Mar 24, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_CTRL_SRC_MODELLEARNER_H_
#define AFFW_CTRL_SRC_MODELLEARNER_H_

#include <vector>
#include <string>

#include "affw/Config.h"
#include "affw/mapping/DataMapper.h"
#include "affw/affw_common.h"

namespace affw {

class ModelLearner {
public:
	ModelLearner(Config& config, DataMapper* dataMapper);
	virtual ~ModelLearner();
	virtual void addData(const Vector& state, const Vector& target, const Vector& action, const Vector& actionComp, const Vector& nextState) = 0;
	virtual Vector getActionCompensation(const Vector& state, const Vector& target) = 0;
	virtual void write(const std::string& folder) = 0;
private:
};

} /* namespace affw */

#endif /* AFFW_CTRL_SRC_MODELLEARNER_H_ */
