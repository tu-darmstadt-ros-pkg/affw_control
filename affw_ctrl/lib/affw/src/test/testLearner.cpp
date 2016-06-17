/*
 * testLearner.cpp
 *
 *  Created on: May 18, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include <string>
#include <iostream>
#include <fstream>

#include "../affw.h"


int main(int argc, char **argv) {
//	std::string folder = "/home/geforce/git/matlab-thesis/2016-05-17_19-18-12_circle_1.3_5it_lwpr_10nFrames_bot6";
//
//	std::fstream f(folder + "/affw.csv");

	affw::Config config;
	config.read("affw.cfg");
	config.print();

	affw::KTermStateTarget2ActionCompMapper mapper(config);
	affw::FANNLearner learner(config);

	affw::Vector state(3);
	affw::Vector target(3);
	affw::Vector action(3);
	affw::Vector actionComp(3);
	affw::Vector nextState(3);
	affw::Vector learnerDebug(0);
	affw::Vector y(0);
	mapper.getOutput(state, target, action, actionComp, nextState, y);
	for(int i=0;i<10;i++)
		learner.addData(state, target, action, actionComp, nextState, y);

	for(int i=0;i<10;i++)
		learner.getActionCompensation(state, target, learnerDebug);

	return 0;
}
