/*
 * otl_rls_osn.cpp
 *
 *  Created on: Jul 14, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/otl/otl_rls_osn.h"

namespace affw {

RLSESN::RLSESN() {
}

RLSESN::~RLSESN() {
}

void RLSESN::train(const VectorXd &input, const VectorXd &output)
{
	VectorXd state;
    res.update(input);
    res.getState(state);
    rls.train(state, output);

}

void RLSESN::predict(const VectorXd &input, VectorXd &prediction,
                  VectorXd &prediction_variance)
{
	VectorXd state;
	predictionRes.update(input);
	predictionRes.getState(state);
	rls.predict(state, prediction, prediction_variance);
}

void RLSESN::reset() {
	res.reset();
	predictionRes.reset();
	rls.reset();
}

void RLSESN::save(std::string filename) {
}

void RLSESN::load(std::string filename) {
}


void RLSESN::init(unsigned int input_dim,
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

					double delta,
					double lambda,
					double noise
					)
{
	res.init(input_dim, output_dim,reservoir_size,
            input_weight, output_feedback_weight,
            activation_function,
            leak_rate, connectivity, spectral_radius,
            use_inputs_in_state,
            random_seed);
	predictionRes = res;

	rls.init(input_dim, output_dim, delta, lambda, noise);
}

} /* namespace affw */
