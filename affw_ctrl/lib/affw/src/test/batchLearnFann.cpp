/*
 * batchLearnFann.cpp
 *
 *  Created on: May 27, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include <iostream>
#include <fstream>
#include <fann.h>
#include <cstring>

int main(int argc, char **argv) {
	if(argc != 3)
	{
		std::cout << "Invalid num args" << std::endl;
		return 1;
	}
	const char* in_file = argv[1];
	const char* out_file = argv[2];

	struct fann_train_data *train_data = fann_read_train_from_file(in_file);
	struct fann_train_data *test_data = fann_read_train_from_file(out_file);


	const unsigned int num_input = train_data->num_input;
	const unsigned int num_output = train_data->num_output;
	const unsigned int num_layers = 3;
	const unsigned int num_neurons_hidden = 10;

	struct fann *ann;
	unsigned int max_neurons = 1000;
	unsigned int neurons_between_reports = 100;
	const float desired_error = (const float) 1e-10;

	int type = 1;

	if(type == 0)
	{
		ann = fann_create_shortcut(2, fann_num_input_train_data(train_data), fann_num_output_train_data(train_data));
		fann_cascadetrain_on_data(ann, train_data, max_neurons, neurons_between_reports, desired_error);

	} else {

		ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);
		fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
		fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);
//		fann_set_training_algorithm(ann, FANN_TRAIN_RPROP);
		fann_set_training_algorithm(ann, FANN_TRAIN_QUICKPROP);
//		fann_set_training_algorithm(ann, FANN_TRAIN_INCREMENTAL);
		fann_set_train_error_function(ann, FANN_ERRORFUNC_LINEAR);

//		fann_set_weight(ann, 0, 2, 0);
//		fann_set_weight(ann, 1, 2, 0);
		fann_set_activation_steepness_output(ann, 1.5);
		fann_set_learning_rate(ann, 0.1);

//		fann_train_on_data(ann, train_data, max_neurons, neurons_between_reports, desired_error);
		fann_train_epoch(ann, train_data);

	}

	std::cout << train_data->num_data << " " << train_data->num_input << " "
			<< train_data->num_output << std::endl;

	printf("Mean Square Error: %f\n", fann_get_MSE(ann));

	struct fann_connection *con;   /* weight matrix */
	unsigned int connum;           /* connections number */
	size_t i;
	connum = fann_get_total_connections(ann);
	if (connum == 0) {
		fprintf(stderr, "Error: connections count is 0\n");
		return EXIT_FAILURE;
	}

	con = (struct fann_connection*) calloc(connum, sizeof(*con));
	if (con == NULL) {
		fprintf(stderr, "Error: unable to allocate memory\n");
		return EXIT_FAILURE;
	}

	/* Get weight matrix */
	fann_get_connection_array(ann, con);

	/* Print weight matrix */
	for (i = 0; i < connum; ++i) {
		printf("weight from %u to %u: %f\n", con[i].from_neuron,
			   con[i].to_neuron, con[i].weight);
	}

	free(con);

	fann_print_connections(ann);


	fann_type output[test_data->num_data][test_data->num_output];
	for(int i=0;i<test_data->num_data;i++)
	{
		fann_type *in = test_data->input[i];
		fann_type *out = fann_run(ann, in);
		for(int j=0;j<test_data->num_output;j++)
			output[i][j] = out[j];
//		memcpy(output[i], out, (test_data->num_output*sizeof(fann_type)));
	}

	std::ofstream myfile;
	myfile.open ("/tmp/input");
	for(int i=0;i<test_data->num_data;i++)
	{
		for(int j=0;j<test_data->num_input;j++)
		{
			myfile << test_data->input[i][j] << " ";
		}
		myfile << std::endl;
	}
	myfile.close();

	myfile.open ("/tmp/output");
	for(int i=0;i<test_data->num_data;i++)
	{
		for(int j=0;j<test_data->num_output;j++)
		{
			myfile << test_data->output[i][j] << " ";
		}
		for(int j=0;j<test_data->num_output;j++)
		{
			myfile << output[i][j] << " ";
		}
		myfile << std::endl;
	}
	myfile.close();

	fann_reset_MSE(ann);
	for(i = 0 ; i != test_data->num_data ; i++ ) {
	  fann_test(ann, test_data->input[i], test_data->output[i]);
	}
	printf("Mean Square Error: %f\n", fann_get_MSE(ann));

	fann_destroy_train(test_data);
	fann_destroy_train(train_data);
	fann_destroy(ann);

	return 0;
}


