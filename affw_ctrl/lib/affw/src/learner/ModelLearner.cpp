/*
 * ModelLearner.cpp
 *
 *  Created on: Mar 26, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "ModelLearner.h"
#include <iostream>

namespace affw {


ModelLearner::ModelLearner(Config& config)
{
	this->config = config;
	int actionDim = config.getInt("actionDim", 1);
	int stateDim = config.getInt("stateDim", 1);

	upperInputBounds = config.getDoubleVector("upperInputBounds", upperInputBounds);
	if(upperInputBounds.empty())
	{
		std::cout << "No upper input bounds set!" << std::endl;
		for(int i=0;i<stateDim;i++) upperInputBounds.push_back(1);
	} else if(upperInputBounds.size() != stateDim)
	{
		std::cout << "Dimensions of upper input bounds do not match: " << upperInputBounds.size() << " != " << stateDim << std::endl;
		upperInputBounds.clear();
		for(int i=0;i<stateDim;i++) upperInputBounds.push_back(1);
	}

	upperOutputBounds = config.getDoubleVector("upperOutputBounds", upperOutputBounds);
	if(upperOutputBounds.empty())
	{
		std::cout << "No upper output bounds set!" << std::endl;
		for(int i=0;i<actionDim;i++) upperOutputBounds.push_back(1);
	} else if(upperOutputBounds.size() != actionDim)
	{
		std::cout << "Dimensions of upper output bounds do not match: " << upperOutputBounds.size() << " != " << actionDim << std::endl;
		upperOutputBounds.clear();
		for(int i=0;i<actionDim;i++) upperOutputBounds.push_back(1);
	}
}

ModelLearner::~ModelLearner() {
}


Config& ModelLearner::getConfig()
{
	return this->config;
}

} /* namespace affw */
