/*
 * LWPRLearner.cpp
 *
 *  Created on: Mar 15, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "../../../affw_ctrl/src/learner/LWPRLearner.h"

#include <iostream>
#include <boost/filesystem.hpp>

namespace affw {

LWPR_Learner::LWPR_Learner(Config* cfg)
{
	config = cfg;
	int nFrames = config->getInt("nFrames", 1);
	int actionDim = config->getInt("actionDim", 1);

	cutoff = config->getDouble("cutoff", 0.001);
	k = config->getDouble("k", 1);
	maxComp = config->getDouble("maxComp", 0.5);

	std::string dataFolder = config->getString("dataFolder", "");
	std::string lwpr_config = config->getString("lwpr_config", "lwpr");
	std::string configPath = dataFolder + "/" + lwpr_config + ".bin";
	if(boost::filesystem::exists(configPath))
	{
		model = new LWPR_Object(configPath.c_str());
		std::cout << "Reusing existing model with " << model->nData() << " nData." << std::endl;

		if(model->nOut() != actionDim)
		{
			std::cerr << "Invalid LWPR model: actionDim does not match (" <<model->nOut() << "!=" << actionDim << std::endl;
		}
	} else {
		model = new LWPR_Object(actionDim*(1+nFrames),actionDim);
	}

	// normalization, TODO: move outside, make dynamic
	doubleVec normIn = {5, 5, 30, 5, 5, 30};
	model->normIn(normIn);

	// #### distance metric
	model->setInitAlpha(config->getDouble("initAlpha", 50));

	// meta learning rate
	model->useMeta(config->getBool("useMeta", false));
	model->metaRate(config->getDouble("metaRate", 250));

	// penalty factor (higher -> smoother) default: 1e-6
	model->penalty(config->getDouble("penalty", 1e-2));

	/* Set initial distance metric to 50*(identity matrix) */
	model->setInitD(config->getDouble("initD", 10));


	// #### local regression
	model->wGen(config->getDouble("wGen", 0.2));
}

//LWPR_Learner::LWPR_Learner(int nFrames, int actionDim) :
//		model(actionDim*(1+nFrames),actionDim)
//{
//   cutoff = 0.001;
//   k = 1;
//   maxComp = 0.5;
//
//   // normalization, TODO: move outside, make dynamic
//   doubleVec normIn = {5, 5, 30, 5, 5, 30};
//   model.normIn(normIn);
//
//
//
//}

LWPR_Learner::~LWPR_Learner() {
}


std::vector<double> LWPR_Learner::update(std::vector<double> x, std::vector<double> y)
{
    doubleVec yp = model->update((doubleVec) x, (doubleVec) y);
    return yp;
}


void LWPR_Learner::addData(const Vector& state, const Vector& target, const Vector& action, const Vector& actionComp, const Vector& nextState)
{
	doubleVec x;
	x.insert(x.end(), state.begin(), state.end());
	x.insert(x.end(), target.begin(), target.end());
	doubleVec y(nextState.size(), 0);

	for(int i=0;i<nextState.size();i++) {
		y[i] = actionComp[i];
	}
	for(int i=nextState.size()-1; i>=0;i--) {
		double diff = ( target[i] - nextState[i] );
		double comp = k * diff;
		if(comp > maxComp)
			comp = maxComp;
		if(comp < -maxComp)
			comp = -maxComp;
		y[i] += comp;

//		if(fabsf(diff) > 0.2)
//			break;
//		else
//			std::cout << diff << std::endl;
	}

	try {
		doubleVec yp = model->update((doubleVec) x, (doubleVec) y);
	} catch(LWPR_Exception& e)
	{
		std::cerr << "addData: LWPR error: " << e.getString() << std::endl;
	}

}

Vector LWPR_Learner::getActionCompensation(const Vector& state, const Vector& target)
{
	doubleVec yp;
	doubleVec x;
	x.insert(x.end(), state.begin(), state.end());
	x.insert(x.end(), target.begin(), target.end());
	try {
		yp = model->predict(x, cutoff);
	} catch(LWPR_Exception& e)
	{
		std::cerr << "getActionComp: LWPR error: " << e.getString() << std::endl;
	}
	return yp;
}

void LWPR_Learner::write(const std::string& folder)
{
	std::string lwpr_config = config->getString("lwpr_config", "lwpr");
	model->writeBinary((folder + "/" + lwpr_config + ".bin").c_str());
	model->writeXML((folder + "/" + lwpr_config + ".xml").c_str());
}

}
