/*
 * otl_rls_osn.h
 *
 *  Created on: Jul 14, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_LIB_AFFW_SRC_OTL_OTL_RLS_OSN_H_
#define AFFW_AFFW_CTRL_LIB_AFFW_SRC_OTL_OTL_RLS_OSN_H_

#include <otl_learning_algs.h>
#include <otl_reservoir.h>
#include <otl_rls.h>

namespace affw {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using OTL::Reservoir;
using OTL::RLS;

class RLSESN: public OTL::LearningAlgorithm {
public:
	RLSESN();
	virtual ~RLSESN();

    virtual void train(const VectorXd &state, const VectorXd &output);
    virtual void predict(const VectorXd &state, VectorXd &prediction, VectorXd &prediction_variance);
    virtual void reset();
    virtual void save(std::string filename);
    virtual void load(std::string filename);

    /**
      \brief Sets up the reservoir
      \param input_dim the input dimension
      \param output_dim the output dimension
      \param reservoir_size how big is your reservoir?
      \param input_weight How much to weight your inputs (typically 1)
      \param output_feedback_weight Set this >0 if you want output feedback
                with the specified weighting
      \param activation function only OTLParams::TANH is supported for now.
      \param leak_rate the leak rate (between 0 and 1.0) of the reservoir
                (depends on your application)
      \param connectivity connectivity of the reservoir (between 0 and 1.0).
                Typically small e.g., 0.01 or 0.1
      \param spectral_radius the spectral radius of the reservoir. This should
                be < 1.0 but you can set it higher if you want.
      \param use_inputs_in_state do we want to use the inputs directly in
                the state vector?
      \param random_seed the random seed to initialise the reservoir. The same
                random seed will generate the same reservoir.
      **/
    void init(
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
            unsigned int random_seed,

			double delta = 0.1,
			double lambda = 0.99,
			double noise = 0.0
            );

private:
    Reservoir res;
    Reservoir predictionRes;
    RLS rls;
};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_LIB_AFFW_SRC_OTL_OTL_RLS_OSN_H_ */
