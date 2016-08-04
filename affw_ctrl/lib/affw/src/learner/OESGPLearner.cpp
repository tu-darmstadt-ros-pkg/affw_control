/*
 * OESGPLearner.cpp
 *
 *  Created on: Jun 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/learner/OESGPLearner.h"

namespace affw {

OESGPLearner::OESGPLearner(Config& config)
	: ModelLearner(OESGPLearner::name(), config)
{
	// common parameters
	int actionDim = config.getInt("actionDim", 1);
	int stateDim = config.getInt("stateDim", 1);

    //Reservoir Parameters
    int reservoir_size = 10;
    double input_weight = 1.0;
    double output_feedback_weight = 0.0;
    int activation_function = OTL::Reservoir::TANH;
    double leak_rate = 0.0;
    double connectivity = 0.1;
    double spectral_radius = 0.99;
    bool use_inputs_in_state = false;
    int random_seed = 0;

    //SOGP parameters
    double noise = 0.1;
    double epsilon = 1e-3;
    int capacity = 100;
    double kernel_l = 0.5;
    double kernel_alpha = 1.0;

	reservoir_size = config.getInt(config_prefix + "reservoir_size", reservoir_size);
	leak_rate = config.getDouble(config_prefix + "leak_rate", leak_rate);
	connectivity = config.getDouble(config_prefix + "connectivity", connectivity);
	spectral_radius = config.getDouble(config_prefix + "spectral_radius", spectral_radius);
	use_inputs_in_state = config.getBool(config_prefix + "use_inputs_in_state", use_inputs_in_state);
	random_seed = config.getInt(config_prefix + "random_seed", random_seed);
	noise = config.getDouble(config_prefix + "noise", noise);
	epsilon = config.getDouble(config_prefix + "epsilon", epsilon);
	capacity = config.getInt(config_prefix + "capacity", capacity);
	kernel_l = config.getDouble(config_prefix + "kernel_l", kernel_l);
	kernel_alpha = config.getDouble(config_prefix + "kernel_alpha", kernel_alpha);

    OTL::VectorXd kernel_parameters(2); //gaussian kernel parameters
    kernel_parameters << kernel_l, kernel_alpha; //l, alpha

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
	nData++;
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

Vector OESGPLearner::getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug)
{
    OTL::VectorXd input(state.size());
    for(int i=0;i<state.size();i++)
    	input(i) = state[i] / upperInputBounds[i];

    OTL::VectorXd prediction;
    OTL::VectorXd prediction_variance;

	m_mutex.lock();

	model.predictAsync(input, prediction, prediction_variance);

	learnerDebug.resize(target.size()*3);
	for(int i=0;i<target.size();i++)
	{
		learnerDebug[i] = prediction_variance[i];
		learnerDebug[i+target.size()] = model.getCurrentSize();
		learnerDebug[i+target.size()*2] = model.getActualSpectralRadius();
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

void OESGPLearner::read(const std::string& folder)
{
	model.load(folder + "/oesgp");
}

void OESGPLearner::write(const std::string& folder)
{
	model.save(folder + "/oesgp");
}

} /* namespace affw */
