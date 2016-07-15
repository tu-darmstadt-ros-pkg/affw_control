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

LWPR_Learner::LWPR_Learner(Config& config)
	: ModelLearner(LWPR_Learner::name(), config)
{
	// common parameters
	int actionDim = config.getInt("actionDim", 1);
	int stateDim = config.getInt("stateDim", 1);

	// lwpr parameters
	cutoff = config.getDouble(config_prefix + "cutoff", 0.001);

	// create new model
	model = new LWPR_Object(stateDim,actionDim);

	updateModel();
}

LWPR_Learner::~LWPR_Learner() {
}

void LWPR_Learner::updateModel()
{
	int actionDim = config.getInt("actionDim", 1);
	int stateDim = config.getInt("stateDim", 1);

	// normalization
	if(upperInputBounds.size() == stateDim && upperOutputBounds.size() == actionDim)
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
	model->updateD(config.getBool(config_prefix + "updateD", false));

	model->initLambda(config.getDouble(config_prefix + "initLambda", 0.999));
	model->finalLambda(config.getDouble(config_prefix + "finalLambda", 0.99999));
	model->tauLambda(config.getDouble(config_prefix + "tauLambda", 0.9999));


	// #### local regression
	model->wGen(config.getDouble(config_prefix + "wGen", 0.1));
}

void LWPR_Learner::addData(
		const Vector& state,
		const Vector& target,
		const Vector& action,
		const Vector& actionComp,
		const Vector& nextState,
		const Vector& y)
{
	m_mutex.lock();
	try {
		doubleVec yp = model->update((doubleVec) state, (doubleVec) y);
	} catch(LWPR_Exception& e)
	{
		std::cerr << "addData: LWPR error: " << e.getString() << std::endl;
	}
	m_mutex.unlock();
}

Vector LWPR_Learner::getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug)
{
	doubleVec yp,conf,wMax;

	m_mutex.lock();
	try {
		yp = model->predict(state, conf, wMax, cutoff);
	} catch(LWPR_Exception& e)
	{
		std::cerr << "getActionComp: LWPR error: " << e.getString() << " stateSize:"
				<< state.size() << " targetSize:" << target.size() << " modelInDim:" << model->nIn() << std::endl;
	}
	m_mutex.unlock();

	learnerDebug.insert(learnerDebug.end(), conf.begin(), conf.end());
	learnerDebug.insert(learnerDebug.end(), wMax.begin(), wMax.end());
	learnerDebug.insert(learnerDebug.end(), yp.begin(), yp.end());

	for(int i=0;i<yp.size();i++)
	{
		// cut off at bounds
		yp[i] = fminf(upperOutputBounds[i], yp[i]);
		yp[i] = fmaxf(-upperOutputBounds[i], yp[i]);
	}
	return yp;
}

void LWPR_Learner::read(const std::string& folder)
{
	std::string configPath = folder + "/lwpr.bin";
	if(boost::filesystem::exists(configPath))
	{
		model = new LWPR_Object(configPath.c_str());
		std::cout << "Reusing existing model with " << model->nData() << " nData." << std::endl;
		updateModel();
	}
}

void LWPR_Learner::write(const std::string& folder)
{
	std::cout << "Save LWPR model to " << folder << std::endl;
	m_mutex.lock();
	model->writeBinary((folder + "/lwpr.bin").c_str());
	model->writeXML((folder + "/lwpr.xml").c_str());
	m_mutex.unlock();
}

}
