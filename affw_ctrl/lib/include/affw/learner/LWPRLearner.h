/*
 * LWPRLearner.h
 *
 *  Created on: Mar 15, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_CTRL_SRC_LWPRLEARNER_H_
#define AFFW_CTRL_SRC_LWPRLEARNER_H_

#include <affw/learner/ModelLearner.h>
#include <affw/mapping/DataMapper.h>
#include <string>
#include <vector>
#include <lwpr.hh>

namespace affw {

class LWPR_Learner : public ModelLearner {
public:
	LWPR_Learner(Config& config, DataMapper* dataMapper);
	virtual ~LWPR_Learner();
	void addData(const Vector& state, const Vector& target, const Vector& action, const Vector& actionComp, const Vector& nextState);
	Vector getActionCompensation(const Vector& state, const Vector& target);
	void write(const std::string& folder);
private:

	std::string lwpr_config;

	LWPR_Object* model;
	double cutoff;

	DataMapper* dataMapper;

	std::vector<double> update(std::vector<double>, std::vector<double>);
	std::vector<double> predict(std::vector<double>);
};

}

#endif /* AFFW_CTRL_SRC_LWPRLEARNER_H_ */
