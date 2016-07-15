/*
 * OESGPasync.h
 *
 *  Created on: Jul 14, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_LIB_AFFW_SRC_OTL_OESGPASYNC_H_
#define AFFW_AFFW_CTRL_LIB_AFFW_SRC_OTL_OESGPASYNC_H_

#include <otl_oesgp.h>

namespace affw {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using OTL::Reservoir;

class OESGPasync: public OTL::OESGP {
public:
	OESGPasync();
	virtual ~OESGPasync();
	virtual void init(
	        unsigned int input_dim,
	        unsigned int output_dim,
	        unsigned int reservoir_size,
	        double input_weight,
	        double output_feedback_weight,
	        int activation_function,
	        double leak_rate,
	        double connectivity,
	        double spectral_radius,
	        bool use_inputs_in_state,
	        VectorXd &kernel_parameters,
	        double noise,
	        double epsilon,
	        unsigned int capacity,
	        unsigned int random_seed
	        );
    virtual void updateAsync(const VectorXd &input, const VectorXd &output);
    virtual void predictAsync(const VectorXd &input, VectorXd &prediction, VectorXd &prediction_variance);
private:
    Reservoir predictionRes;
};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_LIB_AFFW_SRC_OTL_OESGPASYNC_H_ */
