/*
 * affw_factory.cpp
 *
 *  Created on: Jul 14, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/affw.h"

affw::ModelLearner* createAffwLearner(std::string& learner_type, affw::Config& config)
{
	affw::ModelLearner* learner = 0;
	if(learner_type == affw::FANNLearner::name())
	{
		learner = new affw::FANNLearner(config);
	} else if(learner_type == affw::FeedbackController::name())
	{
		learner = new affw::FeedbackController(config);
	}  else if(learner_type == affw::LWPR_Learner::name())
	{
		learner = new affw::LWPR_Learner(config);
	} else if(learner_type == affw::OESGPLearner::name())
	{
		learner = new affw::OESGPLearner(config);
	} else if(learner_type == affw::RLSESNLearner::name())
	{
		learner = new affw::RLSESNLearner(config);
	} else if(learner_type == affw::RLSLearner::name())
	{
		learner = new affw::RLSLearner(config);
	} else if(learner_type == affw::SOGPLearner::name())
	{
		learner = new affw::SOGPLearner(config);
	} else if(learner_type == affw::STORKGPLearner::name())
	{
		learner = new affw::STORKGPLearner(config);
	} else if(learner_type == affw::OSVRLearner::name())
	{
		learner = new affw::OSVRLearner(config);
	} else if(learner_type == affw::WrapperLearner::name())
	{
		learner = new affw::WrapperLearner(config);
	} else
	{
		learner = new affw::DummyLearner(config);
	}

	return learner;
}
