/*
 * STORKGPLearner.cpp
 *
 *  Created on: Jun 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/learner/STORKGPLearner.h"

namespace affw {

STORKGPLearner::STORKGPLearner(Config& config)
	: ModelLearner(STORKGPLearner::name(), config)
{
	// common parameters
	int actionDim = config.getInt("actionDim", 1);
	int stateDim = config.getInt("stateDim", 1);

    //parameters for the STORKGP algorithm
    unsigned int tau = 5;  //length of memory
    tau = config.getInt(config_prefix + "tau", tau);

    //kernel parameters
    double l = 0.5;
    double rho = 0.99;
    double alpha = 1.0;

    //SOGP parameters
    double noise = 0.1;
    double epsilon = 1e-3;
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

	m_mutex.lock();

	model.updateAsync(input, output);

	m_mutex.unlock();
}

Vector STORKGPLearner::getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug)
{

    OTL::VectorXd input(state.size());
    for(int i=0;i<state.size();i++)
    	input(i) = state[i] / upperInputBounds[i];

    OTL::VectorXd prediction(target.size());
    OTL::VectorXd prediction_variance(target.size());

	m_mutex.lock();

	model.predictAsync(input, prediction, prediction_variance);

	learnerDebug.resize(target.size()*2);
	for(int i=0;i<target.size();i++)
	{
		learnerDebug[i] = prediction_variance[i];
		learnerDebug[i+target.size()] = model.getCurrentSize();
	}

	m_mutex.unlock();

	Vector v(target.size());
    for(int i=0;i<target.size();i++)
    {
    	v[i] = prediction(i) * upperOutputBounds[i];
    }

	for(int i=0;i<v.size();i++)
	{
		// cut off at bounds
		v[i] = fminf(upperOutputBounds[i], v[i]);
		v[i] = fmaxf(-upperOutputBounds[i], v[i]);
	}
	return v;
}

void STORKGPLearner::read(const std::string& folder)
{
	m_mutex.lock();
	model.load(folder + "/storkgp");
	m_mutex.unlock();
}

void STORKGPLearner::write(const std::string& folder)
{
	m_mutex.lock();
	model.save(folder + "/storkgp");
	m_mutex.unlock();
}

} /* namespace affw */
