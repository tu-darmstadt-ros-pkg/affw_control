/*
 * traj_generator.h
 *
 *  Created on: 26.05.2016
 *      Author: AndreR
 */

#ifndef COMMON_SRC_UTIL_TRAJ_GENERATOR_H_
#define COMMON_SRC_UTIL_TRAJ_GENERATOR_H_

#include "traj_2order.h"

#define MODE_RESET					0
#define MODE_LOCAL_VEL				1
#define MODE_GLOBAL_POS				2
#define MODE_GLOBAL_VEL_AND_ORIENT	3

typedef struct _TrajGenerator
{
	float vMaxXY;
	float aMaxXY;
	float jMaxXY;	// only observed by velocity trajectories
	float vMaxW;
	float aMaxW;
	float jMaxW;

	float dampingXY;
	float dampingW;
	float tShiftXY;
	float tShiftW;

	uint8_t mode;

	float filterGlobalPos[3];
	float filterGlobalVel[3];

	float trajBBGlobalPos[3];
	float trajBBGlobalVel[3];

	float trajGlobalPos[3];
	float trajGlobalVel[3];
	float trajGlobalAcc[3];
	float trajGlobalJerk[3];

	TrajSecOrder2D trajPosXY;
	TrajSecOrder1D trajPosW;

	float trajLocalVel[3];
	float trajLocalAcc[3];
	float trajLocalJerk[3];

	TrajSecOrder2D trajVelXY;
	TrajSecOrder1D trajVelW;
} TrajGenerator;

void TrajGeneratorInit(TrajGenerator* pGen);
void TrajGeneratorReset(TrajGenerator* pGen);
void TrajGeneratorSetState(TrajGenerator* pGen, float* pGlobalPos, float* pGlobalVel);
void TrajGeneratorCreateGlobalPos(TrajGenerator* pGen, float* pPosTarget);
void TrajGeneratorCreateLocalVel(TrajGenerator* pGen, float* pVelTarget);
void TrajGeneratorCreateGlobalVelAndOrient(TrajGenerator* pGen, float* pVelXY, float orient);
void TrajGeneratorStep(TrajGenerator* pGen, float tStep);

#endif /* COMMON_SRC_UTIL_TRAJ_GENERATOR_H_ */
