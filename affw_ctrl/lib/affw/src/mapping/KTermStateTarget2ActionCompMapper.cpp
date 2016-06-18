/*
 * KTermStateTarget2ActionCompMapper.cpp
 *
 *  Created on: May 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/mapping/KTermStateTarget2ActionCompMapper.h"
#include <iostream>

namespace affw {

KTermStateTarget2ActionCompMapper::KTermStateTarget2ActionCompMapper(Config& config)
	: DataMapper(config)
{

	std::string config_prefix = "kterm_st2ac.";
	int actionDim = config.getInt("actionDim", 1);

	for(int i=0;i<actionDim;i++)
	{
		k.push_back(1);
	}

	maxComp = config.getDoubleVector("upperOutputBounds", maxComp);
	if(maxComp.empty())
	{
		std::cout << "No upper output bounds set!" << std::endl;
		for(int i=0;i<actionDim;i++) maxComp.push_back(1);
	} else if(maxComp.size() != actionDim)
	{
		std::cout << "Dimensions of upper output bounds do not match: " << maxComp.size() << " != " << actionDim << std::endl;
		maxComp.clear();
		for(int i=0;i<actionDim;i++) maxComp.push_back(1);
	}
	std::vector<double> c_k = config.getDoubleVector(config_prefix + "k", k);
	if(c_k.size() != actionDim)
	{
		std::cout << "k has wrong dimension: " << c_k.size() << std::endl;
	} else {
		k = c_k;
	}
}

KTermStateTarget2ActionCompMapper::~KTermStateTarget2ActionCompMapper() {
}

void KTermStateTarget2ActionCompMapper::getOutput(const Vector& state, const Vector& target, const Vector& action, const Vector& actionComp, const Vector& nextState, std::vector<double>& oVec)
{
	int dim = nextState.size();
	if(dim != target.size()
		|| dim != action.size()
		|| dim != actionComp.size()
		|| dim != maxComp.size()
		|| dim != k.size())
	{
		std::cerr << "Invalid dimensions!" << std::endl;
		return;
	}

	oVec.resize(dim, 0);

	for(int i=0;i<dim;i++) {
		oVec[i] = actionComp[i];
	}
	for(int i=dim-1; i>=0;i--) {
		double diff = ( target[i] - nextState[i] );
		if(diff > 1)
			diff = 1;
		if(diff < -1)
			diff = -1;
		double comp = k[i] * diff;
		oVec[i] += comp;
		if(oVec[i] > maxComp[i])
			oVec[i] = maxComp[i];
		if(oVec[i] < -maxComp[i])
			oVec[i] = -maxComp[i];
	}
}

} /* namespace affw */
