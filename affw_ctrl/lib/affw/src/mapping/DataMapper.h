/*
 * DataMapper.h
 *
 *  Created on: May 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_SRC_MAPPING_DATAMAPPER_H_
#define AFFW_AFFW_CTRL_SRC_MAPPING_DATAMAPPER_H_

#include "../affw_common.h"
#include "../Config.h"


namespace affw {

class DataMapper {
public:
	DataMapper(Config& config);
	virtual ~DataMapper() = 0;
	virtual void getOutput(const Vector& state, const Vector& target, const Vector& action, const Vector& actionComp, const Vector& nextState, std::vector<double>& oVec) = 0;
};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_SRC_MAPPING_DATAMAPPER_H_ */
