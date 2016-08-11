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
	modelPerDim = true;

	// RLS parameters
	double delta = 1; // δ > 100*σ²: σ=0.1 -> δ = 1
    double lambda = 0.99; // forgetting factor
    double noise = 0;

    delta = config.getDouble(config_prefix + "delta", delta);
    lambda = config.getDouble(config_prefix + "lambda", lambda);
    noise = config.getDouble(config_prefix + "noise", noise);
    modelPerDim = config.getBool(config_prefix + "modelPerDim", modelPerDim);

	try {
		if(modelPerDim)
		{
			for(int i=0;i<actionDim;i++)
			{
				OTL::RLS* rls = new OTL::RLS();
				rls->init(stateDim, 1, delta, lambda, noise);
				model.push_back(rls);
			}
		} else {
			model.push_back(new OTL::RLS());
			model[0]->init( stateDim, actionDim, delta, lambda, noise);
		}
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
	nData++;
    OTL::VectorXd input(state.size());
    OTL::VectorXd output(y.size());

	//create the input and output
    for(int i=0;i<state.size();i++)
    	input(i) = state[i] / upperInputBounds[i];

    for(int i=0;i<y.size();i++)
    	output(i) = y[i] / upperOutputBounds[i];

	m_mutex.lock();
	try{
		if(modelPerDim)
		{
			for(int i=0;i<y.size();i++)
			{
				OTL::VectorXd out(1);
				out(0) = output(i);
				model[i]->train(input, out);
			}
		} else {
			model[0]->train(input, output);
		}
	} catch (OTL::OTLException &e) {
		e.showError();
	}
	m_mutex.unlock();
}

Vector RLSLearner::getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug)
{
    OTL::VectorXd input(state.size());
    for(int i=0;i<state.size();i++)
    	input(i) = state[i] / upperInputBounds[i];

    OTL::VectorXd prediction(target.size());
    OTL::VectorXd prediction_variance(target.size());

	m_mutex.lock();

	try {
		if(modelPerDim)
		{
			for(int i=0;i<target.size();i++)
			{
				OTL::VectorXd pred(1);
				OTL::VectorXd predVar(1);
				model[i]->predict(input, pred, predVar);
				prediction(i) = pred(0);
				prediction_variance(i) = predVar(0);
			}
		} else {
			model[0]->predict(input, prediction, prediction_variance);
		}
	} catch (OTL::OTLException &e) {
		e.showError();
	}

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
		if(nData < min_nData)
		{
			v[i] = 0;
		}
		// cut off at bounds
		v[i] = fminf(upperOutputBounds[i], v[i]);
		v[i] = fmaxf(-upperOutputBounds[i], v[i]);
	}

	return v;
}

void RLSLearner::read(const std::string& folder)
{
	try {
		for(int i=0;model.size();i++)
		{
			model[i]->load(folder + "/rls" + std::to_string(i));
		}
	} catch (OTL::OTLException &e) {
		e.showError();
	}

	nData = 10000;
}

void RLSLearner::write(const std::string& folder)
{
	try {
		for(int i=0;model.size();i++)
		{
			model[i]->load(folder + "/rls" + std::to_string(i));
		}
	} catch (OTL::OTLException &e) {
		e.showError();
	}
}

} /* namespace affw */
