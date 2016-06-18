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

#include "../Config.h"
#include "../mapping/DataMapper.h"
#include "../affw_common.h"

namespace affw {

class ModelLearner {
public:
	ModelLearner(Config& config);
	virtual ~ModelLearner();
	virtual void addData(	const Vector& state,
							const Vector& target,
							const Vector& action,
							const Vector& actionComp,
							const Vector& nextState,
							const Vector& y) = 0;
	virtual Vector getActionCompensation(const Vector& state, const Vector& target, Vector& learnerDebug) = 0;
	virtual void read(const std::string& folder) = 0;
	virtual void write(const std::string& folder) = 0;

	Config& getConfig();
protected:

	std::vector<double> upperInputBounds;
	std::vector<double> upperOutputBounds;
	Config config;
};

} /* namespace affw */

#endif /* AFFW_CTRL_SRC_MODELLEARNER_H_ */
