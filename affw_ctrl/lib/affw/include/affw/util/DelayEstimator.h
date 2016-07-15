/*
 * DelayEstimator.h
 *
 *  Created on: Jun 18, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_LIB_AFFW_SRC_UTIL_DELAYESTIMATOR_H_
#define AFFW_AFFW_CTRL_LIB_AFFW_SRC_UTIL_DELAYESTIMATOR_H_

#include <boost/circular_buffer.hpp>

namespace affw {

class DelayEstimator {
public:
	DelayEstimator(double timeHorizon, double precision, double minDelay, double maxDelay);
	virtual ~DelayEstimator();

	void update(double t, double x, double y);

	double delay;

private:
	bool delay_by_cross_correlation(double& delay);

	double timeHorizon;
	double precision;
	double maxDelay;
	double minDelay;

	boost::circular_buffer<double> x_buffer, y_buffer;
	double lastTime;

};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_LIB_AFFW_SRC_UTIL_DELAYESTIMATOR_H_ */
