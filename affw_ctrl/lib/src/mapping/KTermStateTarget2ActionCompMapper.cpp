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
		maxComp.push_back(1);
	}

	std::vector<double> c_maxComp = config.getDoubleVector("maxCompensation", maxComp);
	if(c_maxComp.size() != actionDim)
	{
		std::cout << "Max comp has wrong dimension: " << c_maxComp.size() << std::endl;
	} else {
		maxComp = c_maxComp;
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

void KTermStateTarget2ActionCompMapper::getInput(const Vector& state, const Vector& target, std::vector<double>& oVec)
{
	oVec.clear();
	oVec.reserve(state.size()+target.size());
	oVec.insert(oVec.end(), state.begin(), state.end());
	oVec.insert(oVec.end(), target.begin(), target.end());
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

	oVec.clear();
	oVec.resize(dim);

	for(int i=0;i<dim;i++) {
		oVec[i] = actionComp[i];
	}
	for(int i=dim-1; i>=0;i--) {
		double diff = ( target[i] - nextState[i] );
		double comp = k[i] * diff;
		oVec[i] += comp;
		if(oVec[i] > maxComp[i])
			oVec[i] = maxComp[i];
		if(oVec[i] < -maxComp[i])
			oVec[i] = -maxComp[i];
	}
}

} /* namespace affw */
