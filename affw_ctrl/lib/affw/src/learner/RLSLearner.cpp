/*
 * RLSLearner.cpp
 *
 *  Created on: Jul 14, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/learner/RLSLearner.h"

namespace affw {

RLSLearner::RLSLearner(Config& config)
		: ModelLearner(RLSLearner::name(), config)
{
	// common parameters
	int actionDim = config.getInt("actionDim", 1);
	int stateDim = config.getInt("stateDim", 1);

	// RLS parameters
	double delta = 0.1;
    double lambda = 0.99;
    double noise = 1e-3;

    delta = config.getDouble(config_prefix + "delta", delta);
    lambda = config.getDouble(config_prefix + "lambda", lambda);
    noise = config.getDouble(config_prefix + "noise", noise);

	try {
		model.init( stateDim, actionDim, delta, lambda, noise);
	} catch (OTL::OTLException &e) {
		e.showError();
	}
}

RLSLearner::~RLSLearner() {
}

void RLSLearner::addData(
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

Vector RLSLearner::getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug)
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

void RLSLearner::read(const std::string& folder)
{
	model.load(folder + "/rls");
}

void RLSLearner::write(const std::string& folder)
{
	model.save(folder + "/rls");
}

} /* namespace affw */
