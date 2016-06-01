/*
 * LWPRLearner.cpp
 *
 *  Created on: Mar 15, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/learner/LWPRLearner.h"

#include <iostream>
#include <boost/filesystem.hpp>
#include <vector>

namespace affw {

LWPR_Learner::LWPR_Learner(Config& config, DataMapper* dm)
	: ModelLearner(config, dm)
{
	// common parameters
	dataMapper = dm;
	int nFrames = config.getInt("nFrames", 1);
	int actionDim = config.getInt("actionDim", 1);

	// lwpr parameters
	std::string config_prefix = "lwpr.";
	cutoff = config.getDouble(config_prefix + "cutoff", 0.001);
	lwpr_config = "lwpr";

	// create new model
	model = new LWPR_Object(actionDim*(1+nFrames),actionDim);

	// normalization
	if(upperInputBounds.size() == actionDim*(1+nFrames) && upperOutputBounds.size() == actionDim)
	{
		model->normIn(upperInputBounds);
		model->normOut(upperOutputBounds);
	} else {
		std::cout << "Invalid normalization." << std::endl;
	}

	// #### distance metric
	model->setInitAlpha(config.getDouble(config_prefix + "initAlpha", 100));

	// meta learning rate
	model->useMeta(config.getBool(config_prefix + "useMeta", false));
	model->metaRate(config.getDouble(config_prefix + "metaRate", 100));

	// penalty factor (higher -> smoother) default: 1e-6
	model->penalty(config.getDouble(config_prefix + "penalty", 1e-6));

	/* Set initial distance metric to 50*(identity matrix) */
	model->setInitD(config.getDouble(config_prefix + "initD", 50));


	// #### local regression
	model->wGen(config.getDouble(config_prefix + "wGen", 0.1));
}

LWPR_Learner::~LWPR_Learner() {
}


void LWPR_Learner::addData(
		const Vector& state,
		const Vector& target,
		const Vector& action,
		const Vector& actionComp,
		const Vector& nextState,
			  Vector& y)
{
	Vector x;
	dataMapper->getInput(state, target, x);
	dataMapper->getOutput(state, target, action, actionComp, nextState, y);

	try {
		doubleVec yp = model->update((doubleVec) x, (doubleVec) y);
	} catch(LWPR_Exception& e)
	{
		std::cerr << "addData: LWPR error: " << e.getString() << std::endl;
	}
}

Vector LWPR_Learner::getActionCompensation(const Vector& state, const Vector& target, Vector& learnerDebug)
{
	doubleVec x,yp,wMax;
	dataMapper->getInput(state, target, x);

	try {
		yp = model->predict(x, learnerDebug, wMax, cutoff);
	} catch(LWPR_Exception& e)
	{
		std::cerr << "getActionComp: LWPR error: " << e.getString()
				<< state.size() << " " << target.size() << " " << x.size() << std::endl;
	}

	learnerDebug.insert(learnerDebug.end(), wMax.begin(), wMax.end());

	for(int i=0;i<yp.size();i++)
	{
		yp[i] = fminf(upperOutputBounds[i], yp[i]);
		yp[i] = fmaxf(-upperOutputBounds[i], yp[i]);
	}
	return yp;
}

void LWPR_Learner::read(const std::string& folder)
{
	std::string configPath = folder + "/" + lwpr_config + ".bin";
	if(boost::filesystem::exists(configPath))
	{
		model = new LWPR_Object(configPath.c_str());
		std::cout << "Reusing existing model with " << model->nData() << " nData." << std::endl;
	}
//	if(model->nOut() != actionDim)
//	{
//		std::cerr << "Invalid LWPR model: actionDim does not match (" <<model->nOut() << "!=" << actionDim << std::endl;
//	}
}

void LWPR_Learner::write(const std::string& folder)
{
	std::cout << "Save LWPR model to " << folder << std::endl;
	model->writeBinary((folder + "/" + lwpr_config + ".bin").c_str());
	model->writeXML((folder + "/" + lwpr_config + ".xml").c_str());
}

}
