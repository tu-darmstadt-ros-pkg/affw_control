/*
 * lwprPerfTest.cpp
 *
 *  Created on: Jul 22, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include <lwpr.hh>
#include <iostream>
#include <chrono>

int main(int argc, char **argv) {
	const char* filename = "lwpr.bin";
	LWPR_Object m(filename);

	doubleVec input(6, 0);
	doubleVec output(3);
	double cutoff = 0.1;

	int iters = 10;

	std::cout << m.numRFS(0) << " " << m.numRFS(1) << " " << m.numRFS(2) << std::endl;

	std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
	double s = 0;
	for(int i=0;i<iters;i++)
	{
		output = m.predict(input, cutoff);
		s += output[0];
	}
	std::chrono::high_resolution_clock::time_point stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>( stop - start ).count();

	std::cout << s << " " << (duration / iters) << "μs" << std::endl;

	return 0;
}
