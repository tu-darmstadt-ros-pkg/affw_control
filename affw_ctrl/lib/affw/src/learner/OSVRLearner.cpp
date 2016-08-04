/*
 * OSVRLearner.cpp
 *
 *  Created on: Jul 20, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/learner/OSVRLearner.h"

namespace affw {

OSVRLearner::OSVRLearner(Config& config)
	: ModelLearner(OSVRLearner::name(), config)
{
	// common parameters
	int actionDim = config.getInt("actionDim", 1);
	int stateDim = config.getInt("stateDim", 1);

	for(int i=0;i<actionDim;i++)
	{
		onlinesvr::OnlineSVR* SVR = new onlinesvr::OnlineSVR();
		models.push_back(SVR);

		SVR->SetC(1);
		SVR->SetEpsilon(0.01);
		SVR->SetKernelType(onlinesvr::OnlineSVR::KERNEL_RBF);
		SVR->SetKernelParam(30);
		SVR->SetVerbosity(onlinesvr::OnlineSVR::VERBOSITY_NO_MESSAGES);
	}
}

OSVRLearner::~OSVRLearner() {
}
void OSVRLearner::addData(
		const Vector& state,
		const Vector& target,
		const Vector& action,
		const Vector& actionComp,
		const Vector& nextState,
		const Vector& y)
{
	nData++;
	onlinesvr::Vector<double> input(state.size());
	for(int i=0;i<state.size();i++)
		input.Add(state[i]);

	m_mutex.lock();

	for(int i=0;i<y.size();i++)
	{
		models[i]->Train(&input, y[i]);
		// Forget some samples
		onlinesvr::Vector<int>* RemainingSamples = models[i]->GetRemainingSetIndexes()->Clone();
		if(RemainingSamples->GetLength() > 0)
			models[i]->Forget(RemainingSamples);
	}
	m_mutex.unlock();
}

Vector OSVRLearner::getActionCompensation(const Vector& state, const Vector& target, const Vector& preState, Vector& learnerDebug)
{
	onlinesvr::Vector<double> input(state.size());
	for(int i=0;i<state.size();i++)
		input.Add(state[i]);

	Vector v(target.size());

	m_mutex.lock();

	for(int i=0;i<target.size();i++)
	{
		v[i] = models[i]->Predict(&input) * upperOutputBounds[i];
//		learnerDebug[i] = models[i]->get
	}

	m_mutex.unlock();

	for(int i=0;i<v.size();i++)
	{
		if(nData < min_nData)
		{
			v[i] = 0;
		}
		// cut off at bounds
		v[i] = fminf(upperOutputBounds[i], v[i]);
		v[i] = fmaxf(-upperOutputBounds[i], v[i]);
	}

	return v;
}

void OSVRLearner::read(const std::string& folder)
{
	m_mutex.lock();
	char filename[folder.size() + 10];
	for(int i=0;i<models.size();i++)
	{
		sprintf(filename, "%s/osvr_%d", folder.c_str(), i);
		models[i]->LoadOnlineSVR(filename);
	}
	m_mutex.unlock();
}

void OSVRLearner::write(const std::string& folder)
{
	m_mutex.lock();
	char filename[folder.size() + 10];
	for(int i=0;i<models.size();i++)
	{
		sprintf(filename, "%s/osvr_%d", folder.c_str(), i);
		models[i]->SaveOnlineSVR(filename);
	}
	m_mutex.unlock();
}

} /* namespace affw */
