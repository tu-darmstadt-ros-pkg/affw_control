/*
 * affw.h
 *
 *  Created on: Jun 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_LIB_INCLUDE_AFFW_AFFW_H_
#define AFFW_AFFW_CTRL_LIB_INCLUDE_AFFW_AFFW_H_

#include "affw_common.h"

#include "Config.h"

#include "learner/DummyLearner.h"
#include "learner/LWPRLearner.h"
#include "learner/FANNLearner.h"
#include "learner/ModelLearner.h"
#include "learner/FeedbackController.h"
#include "learner/WrapperLearner.h"
#include "learner/OESGPLearner.h"
#include "learner/STORKGPLearner.h"
#include "learner/SOGPLearner.h"
#include "learner/RLSLearner.h"
#include "learner/RLSESNLearner.h"

#include "mapping/KTermStateTarget2ActionCompMapper.h"

affw::ModelLearner* createAffwLearner(std::string& learner_type, affw::Config& config);


#endif /* AFFW_AFFW_CTRL_LIB_INCLUDE_AFFW_AFFW_H_ */
