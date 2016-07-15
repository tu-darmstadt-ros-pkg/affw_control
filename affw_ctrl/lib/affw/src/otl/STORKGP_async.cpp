/*
 * STORKGP_async.cpp
 *
 *  Created on: Jul 14, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/otl/STORKGP_async.h"

namespace affw {

STORKGPasync::STORKGPasync() {
}

STORKGPasync::~STORKGPasync() {
}


void STORKGPasync::init(
        unsigned int input_dim,
        unsigned int output_dim,
        unsigned int tau,
        int kernel_type,
        VectorXd &kernel_parameters,
        double noise,
        double epsilon,
        unsigned int capacity
        )
{
	STORKGP::init(input_dim, output_dim, tau, kernel_type, kernel_parameters,
			noise, epsilon, capacity);

    predictionWindow.init(input_dim, output_dim, tau);
}

void STORKGPasync::updateAsync(const VectorXd &input, const VectorXd &output)
{
	this->update(input);
	this->train(output);
}

void STORKGPasync::predictAsync(const VectorXd &input, VectorXd &prediction, VectorXd &prediction_variance)
{
    this->predictionWindow.update(input);

    VectorXd currentState, state;
    // save current state
    this->getState(currentState);
    // get prediction state
    this->predictionWindow.getState(state);
    // set it for prediction
    this->setState(state);
    // do the prediction
    this->predict(prediction, prediction_variance);
    // recover current state
    this->setState(currentState);
}

} /* namespace affw */
