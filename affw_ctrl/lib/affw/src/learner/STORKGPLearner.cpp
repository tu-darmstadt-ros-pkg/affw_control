/*
 * STORKGPLearner.cpp
 *
 *  Created on: Jun 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/learner/STORKGPLearner.h"

namespace affw {

STORKGPLearner::STORKGPLearner(Config& config)
	: ModelLearner(config)
{
	// common parameters
	int actionDim = config.getInt("actionDim", 1);
	int stateDim = config.getInt("stateDim", 1);

    //parameters for the STORKGP algorithm
    unsigned int tau = 20;  //length of memory

    //kernel parameters
    double l = 0.5;
    double rho = 0.99;
    double alpha = 1.0;

    //SOGP parameters
    double noise = 0.01;
    double epsilon = 1e-4;
    unsigned int capacity = 100;

    OTL::VectorXd kernel_parameters(4);
    //[l rho alpha input_dim]
    kernel_parameters << l, rho, alpha, stateDim;

    try {
            model.init( stateDim, actionDim,
                    tau,
                    OTL::STORKGP::RECURSIVE_GAUSSIAN,
                    kernel_parameters,
                    noise, epsilon, capacity);
	} catch (OTL::OTLException &e) {
		e.showError();
	}

}

STORKGPLearner::~STORKGPLearner() {
}

void STORKGPLearner::addData(
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
    	input(i) = state[i] / upperInputBounds[i];

    for(int i=0;i<y.size();i++)
    	output(i) = y[i] / upperOutputBounds[i];

	//update the OESGP with the input
	model.update(input);

	//train with the true next state
	model.train(output);
}

Vector STORKGPLearner::getActionCompensation(const Vector& state, const Vector& target, Vector& learnerDebug)
{

    OTL::VectorXd input(state.size());
    for(int i=0;i<state.size();i++)
    	input(i) = state[i] / upperInputBounds[i];

    OTL::VectorXd prediction;
    OTL::VectorXd prediction_variance;

	//update the OESGP with the input
	model.update(input);

	//predict the next state
	model.predict(prediction, prediction_variance);

	Vector v(target.size());
    for(int i=0;i<target.size();i++)
    	v[i] = prediction(i) * upperOutputBounds[i];

	return v;
}

void STORKGPLearner::read(const std::string& folder)
{
	model.load(folder + "/oesgp");
}

void STORKGPLearner::write(const std::string& folder)
{
	model.save(folder + "/oesgp");
}

} /* namespace affw */
