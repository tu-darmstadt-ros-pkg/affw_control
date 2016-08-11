/*
 * affw_factory.cpp
 *
 *  Created on: Jul 14, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/affw.h"

affw::ModelLearner* createAffwLearner(std::string& learner_type,
		affw::Config& config) {
	affw::ModelLearner* learner = 0;
	if (learner_type == affw::FeedbackController::name()) {
		learner = new affw::FeedbackController(config);
	}
#ifdef AFFW_LWPR
	else if(learner_type == affw::LWPR_Learner::name())
	{
		learner = new affw::LWPR_Learner(config);
	} else if(learner_type == affw::WrapperLearner::name())
	{
		learner = new affw::WrapperLearner(config);
	}
#endif

#ifdef AFFW_OTL
	else if (learner_type == affw::OESGPLearner::name()) {
		learner = new affw::OESGPLearner(config);
	} else if (learner_type == affw::RLSESNLearner::name()) {
		learner = new affw::RLSESNLearner(config);
	} else if (learner_type == affw::RLSLearner::name()) {
		learner = new affw::RLSLearner(config);
	} else if (learner_type == affw::SOGPLearner::name()) {
		learner = new affw::SOGPLearner(config);
	} else if (learner_type == affw::STORKGPLearner::name()) {
		learner = new affw::STORKGPLearner(config);
	}
#endif

#ifdef AFFW_OSVR
	else if (learner_type == affw::OSVRLearner::name()) {
		learner = new affw::OSVRLearner(config);
	}
#endif
#ifdef AFFW_FANN
	else if (learner_type == affw::FANNLearner::name()) {
		learner = new affw::FANNLearner(config);
	}
#endif
	else {
		learner = new affw::DummyLearner(config);
	}

	return learner;
}
