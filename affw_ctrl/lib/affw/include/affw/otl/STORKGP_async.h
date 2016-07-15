/*
 * STORKGP_async.h
 *
 *  Created on: Jul 14, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_LIB_AFFW_SRC_OTL_STORKGP_ASYNC_H_
#define AFFW_AFFW_CTRL_LIB_AFFW_SRC_OTL_STORKGP_ASYNC_H_

#include <otl_storkgp.h>

namespace affw {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using OTL::Window;

class STORKGPasync: public OTL::STORKGP {
public:
	STORKGPasync();
	virtual ~STORKGPasync();
    virtual void init(
            unsigned int input_dim,
            unsigned int output_dim,
            unsigned int tau,
            int kernel_type,
            VectorXd &kernel_parameters,
            double noise,
            double epsilon,
            unsigned int capacity
            );
    virtual void updateAsync(const VectorXd &input, const VectorXd &output);
    virtual void predictAsync(const VectorXd &input, VectorXd &prediction, VectorXd &prediction_variance);
private:
    Window predictionWindow;
};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_LIB_AFFW_SRC_OTL_STORKGP_ASYNC_H_ */
