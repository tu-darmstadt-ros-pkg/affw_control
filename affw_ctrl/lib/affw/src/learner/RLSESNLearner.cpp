/*
 * RLSESNLearner.cpp
 *
 *  Created on: Jul 14, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/learner/RLSESNLearner.h"

namespace affw {

RLSESNLearner::RLSESNLearner(Config& config)
		: ModelLearner(RLSESNLearner::name(), config)
{
	// common parameters
	int actionDim = config.getInt("actionDim", 1);
	int stateDim = config.getInt("stateDim", 1);

    //Reservoir Parameters
    int reservoir_size = 100;
    double input_weight = 1.0;
    double output_feedback_weight = 0.0;
    int activation_function = OTL::Reservoir::TANH;
    double leak_rate = 0.0;
    double connectivity = 0.1;
    double spectral_radius = 0.99;
    bool use_inputs_in_state = true;
    int random_seed = 0;

	// RLS parameters
	double delta = 0.1;
    double lambda = 0.99;
    double noise = 1e-12;

	reservoir_size = config.getInt(config_prefix + "reservoir_size", reservoir_size);
	std::cout << "reservoir_size=" << reservoir_size << std::endl;
	activation_function = config.getInt(config_prefix + "activation_function", activation_function);
	leak_rate = config.getDouble(config_prefix + "leak_rate", leak_rate);
	connectivity = config.getDouble(config_prefix + "connectivity", connectivity);
	spectral_radius = config.getDouble(config_prefix + "spectral_radius", spectral_radius);
	use_inputs_in_state = config.getBool(config_prefix + "use_inputs_in_state", use_inputs_in_state);
	random_seed = config.getInt(config_prefix + "random_seed", random_seed);

	delta = config.getDouble(config_prefix + "delta", delta);
	lambda = config.getDouble(config_prefix + "lambda", lambda);
	noise = config.getDouble(config_prefix + "noise", noise);

	try {
		model.init( stateDim, actionDim,
				reservoir_size,
                input_weight, output_feedback_weight,
                activation_function,
                leak_rate,
                connectivity, spectral_radius,
                use_inputs_in_state,
                random_seed,
				delta, lambda, noise);
	} catch (OTL::OTLException &e) {
		e.showError();
	}
}

RLSESNLearner::~RLSESNLearner() {
}

void RLSESNLearner::addData(
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

	m_mutex.lock();

	model.train(input, output);

	m_mutex.unlock();
}

Vector RLSESNLearner::getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug)
{
    OTL::VectorXd input(state.size());
    for(int i=0;i<state.size();i++)
    	input(i) = state[i] / upperInputBounds[i];

    OTL::VectorXd prediction;
    OTL::VectorXd prediction_variance;

	m_mutex.lock();

	model.predict(input, prediction, prediction_variance);

	learnerDebug.resize(target.size());
	for(int i=0;i<target.size();i++)
	{
		learnerDebug[i] = prediction_variance[i];
	}

	m_mutex.unlock();

	Vector v(target.size());
    for(int i=0;i<target.size();i++)
    	v[i] = prediction(i) * upperOutputBounds[i];

	for(int i=0;i<v.size();i++)
	{
		// cut off at bounds
		v[i] = fminf(upperOutputBounds[i], v[i]);
		v[i] = fmaxf(-upperOutputBounds[i], v[i]);
	}

	return v;
}

void RLSESNLearner::read(const std::string& folder)
{
	model.load(folder + "/rls-esn");
}

void RLSESNLearner::write(const std::string& folder)
{
	model.save(folder + "/rls-esn");
}

} /* namespace affw */
