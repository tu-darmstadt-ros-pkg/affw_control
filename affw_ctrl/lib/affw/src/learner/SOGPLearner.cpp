/*
 * SOGPLearner.cpp
 *
 *  Created on: Jun 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/learner/SOGPLearner.h"
#include <cmath>

namespace affw {

SOGPLearner::SOGPLearner(Config& config)
	: ModelLearner(SOGPLearner::name(), config)
{
	// common parameters
	int actionDim = config.getInt("actionDim", 1);
	int stateDim = config.getInt("stateDim", 1);

    //kernel parameters
    double l = 0.5;
    double rho = 0.99;
    double alpha = 1.0;

    //SOGP parameters
    double noise = 0.01;
    double epsilon = 1e-3;
    unsigned int capacity = 100;

    OTL::VectorXd kernel_parameters(4);
    //[l rho alpha input_dim]
    kernel_parameters << l, rho, alpha, stateDim;

    kernel.init(stateDim, kernel_parameters);

	int problem_type = OTL::SOGP::REGRESSION;
	int deletion_criteria = OTL::SOGP::NORM;

    try {
            model.init( stateDim, actionDim,
            			kernel,
						noise,
						epsilon,
						capacity,
						problem_type,
						deletion_criteria);
	} catch (OTL::OTLException &e) {
		e.showError();
	}
}

SOGPLearner::~SOGPLearner() {
}

void SOGPLearner::addData(
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
    {
    	input(i) = state[i] / upperInputBounds[i];
    	if(!std::isfinite(input(i)))
    	{
    		std::cerr << "infinite training input detected" << std::endl;
    		input(i) = 0;
    		exit(1);
    	}
    }

    for(int i=0;i<y.size();i++)
    {
    	output(i) = y[i] / upperOutputBounds[i];
    	if(!std::isfinite(output(i)))
    	{
    		std::cerr << "infinite training output detected" << std::endl;
    		output(i) = 0;
    		exit(1);
    	}
    }

	m_mutex.lock();

	model.train(input, output);

	m_mutex.unlock();
}

Vector SOGPLearner::getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug)
{

    OTL::VectorXd input(state.size());
    for(int i=0;i<state.size();i++)
    	input(i) = state[i] / upperInputBounds[i];

    OTL::VectorXd prediction(target.size());
    OTL::VectorXd prediction_variance(target.size());

	m_mutex.lock();

	//update the OESGP with the input
	model.predict(input, prediction, prediction_variance);

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
    	if(!std::isfinite(v[i]))
    	{
    		std::cerr << "infinite prediction detected: " << input << " -> " << prediction << " " << model.getCurrentSize() << std::endl;
    		v[i] = 0;
    		model.save("/tmp/sogp");
    		exit(1);
    	}
    }

	for(int i=0;i<v.size();i++)
	{
		// cut off at bounds
		v[i] = fminf(upperOutputBounds[i], v[i]);
		v[i] = fmaxf(-upperOutputBounds[i], v[i]);
	}

	return v;
}

void SOGPLearner::read(const std::string& folder)
{
	m_mutex.lock();
	model.load(folder + "/sogp");
	m_mutex.unlock();
}

void SOGPLearner::write(const std::string& folder)
{
	m_mutex.lock();
	model.save(folder + "/sogp");
	m_mutex.unlock();
}

} /* namespace affw */
