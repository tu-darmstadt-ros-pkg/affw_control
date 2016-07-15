/*
 * FeedbackController.h
 *
 *  Created on: Mar 26, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_CTRL_SRC_FeedbackController_H_
#define AFFW_CTRL_SRC_FeedbackController_H_

#include "ModelLearner.h"

namespace affw {

class FeedbackController: public ModelLearner {
public:
	FeedbackController(Config& config);
	virtual ~FeedbackController();
	void addData(	const Vector& state,
					const Vector& target,
					const Vector& action,
					const Vector& actionComp,
					const Vector& nextState,
					const Vector& y);
	Vector getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug);
	void read(const std::string& folder);
	void write(const std::string& folder);
	static std::string name() { return "fdbk"; }
	std::string getName();

private:
	Vector actionComp;
	std::vector<double> p;
};

} /* namespace affw */

#endif /* AFFW_CTRL_SRC_FeedbackController_H_ */
