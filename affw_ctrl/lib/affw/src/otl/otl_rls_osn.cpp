/*
 * otl_rls_osn.cpp
 *
 *  Created on: Jul 14, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/otl/otl_rls_osn.h"

namespace affw {

RLSESN::RLSESN() {
	reservoir_size = 0;
}

RLSESN::~RLSESN() {
}

void RLSESN::train(const VectorXd &input, const VectorXd &output)
{
	VectorXd state;
	if(reservoir_size > 0)
	{
		res.update(input);
		res.getState(state);
//		for(int i=0;i<input.size();i++)
//		{
//			if(fabsf(state(state.size()-input.size()+i) - state(i)) > 0.001)
//			{
//				std::cerr << "Invalid state." << std::endl;
//			}
//		}
	} else {
		state = input;
	}
    rls.train(state, output);

}

void RLSESN::predict(const VectorXd &input, VectorXd &prediction,
                  VectorXd &prediction_variance)
{
	VectorXd state;
	if(reservoir_size > 0)
	{
		predictionRes.update(input);
		predictionRes.getState(state);
	} else {
		state = input;
	}
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
	this->reservoir_size = reservoir_size;
	if(reservoir_size > 0)
	{
		res.init(input_dim, output_dim,reservoir_size,
				input_weight, output_feedback_weight,
				activation_function,
				leak_rate, connectivity, spectral_radius,
				use_inputs_in_state,
				random_seed);
		predictionRes = res;
		rls.init(res.getStateSize(), output_dim, delta, lambda, noise);
	} else {
		rls.init(input_dim, output_dim, delta, lambda, noise);
	}
}

} /* namespace affw */
