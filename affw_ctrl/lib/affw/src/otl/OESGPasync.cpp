/*
 * OESGPasync.cpp
 *
 *  Created on: Jul 14, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/otl/OESGPasync.h"

namespace affw {

OESGPasync::OESGPasync() {
}

OESGPasync::~OESGPasync() {
}

void OESGPasync::init(
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
        )
{
	OESGP::init(input_dim, output_dim, reservoir_size,
             input_weight, output_feedback_weight,
             activation_function, leak_rate,
             connectivity, spectral_radius,
             use_inputs_in_state, kernel_parameters,
			 noise, epsilon, capacity,
			 random_seed);

    predictionRes.init(input_dim, output_dim, reservoir_size,
             input_weight, output_feedback_weight,
             activation_function, leak_rate,
             connectivity, spectral_radius,
             use_inputs_in_state, random_seed);
}


void OESGPasync::updateAsync(const VectorXd &input, const VectorXd &output)
{
	this->update(input);
	this->train(output);
}

void OESGPasync::predictAsync(const VectorXd &input, VectorXd &prediction, VectorXd &prediction_variance)
{
    this->predictionRes.update(input);

    VectorXd currentState, state;
    // save current state
    this->getState(currentState);
    // get prediction state
    this->predictionRes.getState(state);
    // set it for prediction
    this->setState(state);
    // do the prediction
    this->predict(prediction, prediction_variance);
    // recover current state
    this->setState(currentState);
}

} /* namespace affw */
