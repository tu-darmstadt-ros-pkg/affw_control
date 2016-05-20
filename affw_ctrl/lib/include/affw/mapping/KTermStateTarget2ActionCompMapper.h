/*
 * KTermStateTarget2ActionCompMapper.h
 *
 *  Created on: May 16, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_AFFW_CTRL_SRC_MAPPING_KTERMSTATETARGET2ACTIONCOMPMAPPER_H_
#define AFFW_AFFW_CTRL_SRC_MAPPING_KTERMSTATETARGET2ACTIONCOMPMAPPER_H_

#include "DataMapper.h"

namespace affw {

class KTermStateTarget2ActionCompMapper: public DataMapper {
public:
	KTermStateTarget2ActionCompMapper(Config& config);
	virtual ~KTermStateTarget2ActionCompMapper();
	void getInput(const Vector& state, const Vector& target, std::vector<double>& oVec);
	void getOutput(const Vector& state, const Vector& target, const Vector& action, const Vector& actionComp, const Vector& nextState, std::vector<double>& oVec);

private:
	std::vector<double> k;
	std::vector<double> maxComp;
};

} /* namespace affw */

#endif /* AFFW_AFFW_CTRL_SRC_MAPPING_KTERMSTATETARGET2ACTIONCOMPMAPPER_H_ */
