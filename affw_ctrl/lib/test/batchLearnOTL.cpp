/*
 * batchLearnFann.cpp
 *
 *  Created on: May 27, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include <iostream>
#include <fstream>
#include <fann.h>
#include "/home/geforce/src/otl/src/libOTL/otl.h"
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


	OTL::RLS rls;

	double delta=0.1, lambda=0.99, noise=0.0;
	rls.init(train_data->num_input, train_data->num_output, delta, lambda, noise);
	OTL::VectorXd train_in(train_data->num_input);
	OTL::VectorXd train_out(train_data->num_output);

	for(int i=0;i<train_data->num_data;i++)
	{
		for(int j=0; j<train_data->num_input;j++)
		{
			train_in(j) = train_data->input[i][j];
		}
		for(int j=0; j<train_data->num_output;j++)
		{
			train_out(j) = train_data->output[i][j];
		}
		rls.train(train_in, train_out);
	}

	OTL::VectorXd test_in(test_data->num_input);
	OTL::MatrixXd pred_out(test_data->num_data, test_data->num_output);
	OTL::VectorXd variance(test_data->num_output);

	for(int i=0;i<test_data->num_data;i++)
	{
		for(int j=0; j<test_data->num_input;j++)
		{
			test_in(j) = test_data->input[i][j];
		}
		OTL::VectorXd pred(test_data->num_output);
		rls.predict(test_in, pred, variance);
		for(int j=0; j<test_data->num_output;j++)
		{
			pred_out(i,j) = pred(j);
		}
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
			myfile << pred_out(i,j) << " ";
		}
		myfile << std::endl;
	}
	myfile.close();


	return 0;
}


