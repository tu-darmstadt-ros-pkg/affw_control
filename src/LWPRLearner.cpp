/*
 * LWPRLearner.cpp
 *
 *  Created on: Mar 15, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "LWPRLearner.h"
#include <iostream>

namespace affw {

LWPR_Learner::LWPR_Learner(int nFrames, int actionDim) :
		model(actionDim*(1+nFrames),actionDim) {

   cutoff = 0.001;

   /* Set initial distance metric to 50*(identity matrix) */
   model.setInitD(50);

   /* Set init_alpha in all elements */
   model.setInitAlpha(500);

   /* Set w_gen to 0.2 */
   model.wGen(0.2);
}

LWPR_Learner::~LWPR_Learner() {
}


std::vector<double> LWPR_Learner::update(std::vector<double> x, std::vector<double> y)
{
    doubleVec yp = model.update((doubleVec) x, (doubleVec) y);
    return yp;
}


void LWPR_Learner::addData(const Vector& state, const Vector& target, const Vector& action, const Vector& actionComp, const Vector& nextState)
{
	doubleVec x,y;
	x.insert(x.end(), state.begin(), state.end());
	x.insert(x.end(), target.begin(), target.end());

	double k = 0.5;
	for(int i=0;i<nextState.size();i++) {
		double diff = ( target[i] - nextState[i] );
		y.push_back(actionComp[i] + k * diff);
		std::cout << diff << " ";
	}
	std::cout << std::endl;

	try {
		doubleVec yp = model.update((doubleVec) x, (doubleVec) y);
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
		yp = model.predict(x, cutoff);
	} catch(LWPR_Exception& e)
	{
		std::cerr << "getActionComp: LWPR error: " << e.getString() << std::endl;
	}
	return yp;
}

}
