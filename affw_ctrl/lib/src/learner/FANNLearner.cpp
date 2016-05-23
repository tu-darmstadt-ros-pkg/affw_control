/*
 * FANNLearner.cpp
 *
 *  Created on: May 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include <affw/learner/FANNLearner.h>
#include <fann.h>
#include <iostream>

namespace affw {

FANNLearner::FANNLearner(Config& config, DataMapper* dataMapper)
	: ModelLearner(config, dataMapper)
{
	this->dataMapper = dataMapper;
	std::string config_prefix = "fann.";

	int nFrames = config.getInt("nFrames", 1);
	int actionDim = config.getInt("actionDim", 1);
	const unsigned int num_input = (1+nFrames) * actionDim;
	const unsigned int num_output = actionDim;
	const unsigned int num_layers = 3;
	const unsigned int num_neurons_hidden = 3;

	ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);

	fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
	fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);
	fann_set_training_algorithm(ann, FANN_TRAIN_INCREMENTAL);
	fann_set_train_error_function(ann, FANN_ERRORFUNC_LINEAR);
}

FANNLearner::~FANNLearner() {
	fann_destroy(ann);
}



void FANNLearner::addData(const Vector& state, const Vector& target, const Vector& action, const Vector& actionComp, const Vector& nextState)
{
	std::vector<double> x,y;
	dataMapper->getInput(state, target, x);
	dataMapper->getOutput(state, target, action, actionComp, nextState, y);
	struct fann_train_data *train_data;
	train_data = fann_create_train(1,x.size(),y.size());
	for(int i=0;i<x.size();i++)
	{
		train_data->input[0][i] = x[i] / upperInputBounds[i];
	}
	for(int i=0;i<y.size();i++)
	{
		train_data->output[0][i] = y[i] / upperOutputBounds[i];
	}

//	unsigned int max_neurons = 30;
//	unsigned int neurons_between_reports = 1;
//	const float desired_error = (const float) 0.01;
//	fann_train_on_data(ann, train_data, max_neurons, neurons_between_reports, desired_error);
	fann_train_epoch(ann, train_data);
	fann_destroy_train(train_data);
}

Vector FANNLearner::getActionCompensation(const Vector& state, const Vector& target)
{
	std::vector<double> x,y;
	dataMapper->getInput(state, target, x);
	fann_type input[x.size()];
	for(int i=0;i<x.size();i++)
		input[i] = (fann_type ) x[i] / upperInputBounds[i];
	fann_type *output = fann_run(ann, input);
	int nOut = fann_get_num_output(ann);
	y.resize(nOut);
	for(int i=0;i<nOut;i++)
		y[i] = output[i] * upperOutputBounds[i];
	return y;
}

void FANNLearner::write(const std::string& folder)
{

}

} /* namespace affw */
