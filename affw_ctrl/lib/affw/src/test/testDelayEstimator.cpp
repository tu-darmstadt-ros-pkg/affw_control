/*
 * testDelayEstimator.cpp
 *
 *  Created on: Jun 18, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/util/DelayEstimator.h"

#include <string>
#include <iostream>
#include <fstream>

int main(int argc, char **argv) {

	std::fstream f_in("/tmp/delay.in");
	if(!f_in.is_open())
	{
		std::cerr << "Could not open input file." << std::endl;
		return 1;
	}
	std::vector<double> t,x,y;

	while(!f_in.eof())
	{
		double tt, xx, yy;
		f_in >> tt >> xx >> yy;
		t.push_back(tt);
		x.push_back(xx);
		y.push_back(yy);
	}

	f_in.close();

	affw::DelayEstimator de(1, 0.0001, -0.5, 0);

	std::vector<double> delays(t.size());
	for(int i=0;i<t.size();i++)
	{
		de.update(t[i], x[i], y[i]);
		delays[i] = de.delay;
	}

	std::ofstream f_out("/tmp/delay.out", std::ios::out);
	if(!f_out.is_open())
	{
		std::cerr << "Could not open output file." << std::endl;
		return 2;
	}
	for(int i=0;i<delays.size();i++)
	{
		f_out << t[i] << " " << x[i] << " " << y[i] << " " << delays[i] << std::endl;
	}
	f_out.close();

	return 0;
}
