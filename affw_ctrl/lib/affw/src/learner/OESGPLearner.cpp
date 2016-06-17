/*
 * OTLLearner.cpp
 *
 *  Created on: Jun 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "OESGPLearner.h"

namespace affw {

OESGPLearner::OESGPLearner(Config& config)
	: ModelLearner(config)
{
	// common parameters
	int actionDim = config.getInt("actionDim", 1);
	int stateDim = config.getInt("stateDim", 1);

    //Reservoir Parameters
    //you can change these to see how it affects the predictions
    int reservoir_size = 100;
    double input_weight = 1.0;
    double output_feedback_weight = 0.0;
    int activation_function = OTL::Reservoir::TANH;
    double leak_rate = 0.9;
    double connectivity = 0.1;
    double spectral_radius = 0.90;
    bool use_inputs_in_state = false;

    OTL::VectorXd kernel_parameters(2); //gaussian kernel parameters
    kernel_parameters << 1.0, 1.0; //l = 1.0, alpha = 1.0

    //SOGP parameters
    double noise = 0.01;
    double epsilon = 1e-3;
    int capacity = 200;

    int random_seed = 0;
    try {
            //Initialise our OESGP
            model.init( stateDim, actionDim, reservoir_size,
                        input_weight, output_feedback_weight,
                        activation_function,
                        leak_rate,
                        connectivity, spectral_radius,
                        use_inputs_in_state,
                        kernel_parameters,
                        noise, epsilon, capacity, random_seed);
	} catch (OTL::OTLException &e) {
		e.showError();
	}

}

OESGPLearner::~OESGPLearner() {
}

void OESGPLearner::addData(
		const Vector& state,
		const Vector& target,
		const Vector& action,
		const Vector& actionComp,
		const Vector& nextState,
		const Vector& y)
{
    OTL::VectorXd input(state.size());
    OTL::VectorXd output(y.size());

	//create the input and output
    for(int i=0;i<state.size();i++)
    	input(i) = state[i];

    for(int i=0;i<y.size();i++)
    	output(i) = y[i];

	//update the OESGP with the input
	model.update(input);

	//train with the true next state
	model.train(output);
}

Vector OESGPLearner::getActionCompensation(const Vector& state, const Vector& target, Vector& learnerDebug)
{

    OTL::VectorXd input(state.size());
    for(int i=0;i<state.size();i++)
    	input(i) = state[i];

    OTL::VectorXd prediction;
    OTL::VectorXd prediction_variance;

	//update the OESGP with the input
	model.update(input);

	//predict the next state
	model.predict(prediction, prediction_variance);

	Vector v(target.size());
    for(int i=0;i<target.size();i++)
    	v[i] = prediction(i);

	return v;
}

void OESGPLearner::read(const std::string& folder)
{
	model.load(folder + "/oesgp");
}

void OESGPLearner::write(const std::string& folder)
{
	model.save(folder + "/oesgp");
}

} /* namespace affw */
