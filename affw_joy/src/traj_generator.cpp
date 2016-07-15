/*
 * traj_generator.c
 *
 *  Created on: 26.05.2016
 *      Author: AndreR
 */

#include "traj_generator.h"
#include "angle_math.h"
#include <string.h>
#include <math.h>


void CtrlUtilTurnLocal2Global(float globalOrientation, float localX, float localY, float* pGlobalX, float* pGlobalY)
{
	float localToGlobalAngle = -M_PI / 2.0f + globalOrientation;
	float sinA = sin(localToGlobalAngle);
	float cosA = cos(localToGlobalAngle);

	// turn to global system
	*pGlobalX = localX * cosA - localY * sinA;
	*pGlobalY = localY * cosA + localX * sinA;
}

void CtrlUtilTurnGlobal2Local(float globalOrientation, float globalX, float globalY, float* pLocalX, float* pLocalY)
{
	float globalToLocalAngle = M_PI / 2.0f - globalOrientation;
	float sinA = sin(globalToLocalAngle);
	float cosA = cos(globalToLocalAngle);

	// turn to local system
	*pLocalX = globalX * cosA - globalY * sinA;
	*pLocalY = globalY * cosA + globalX * sinA;
}

static void changeMode(TrajGenerator* pGen, uint8_t newMode)
{
	if(pGen->mode == newMode)
		return;

	// TODO: add mode changes for GLOBAL_VEL_AND_ORIENT

	switch(pGen->mode)
	{
		case MODE_RESET:
		{
			// out of reset, we only have the filtered data available

			memcpy(pGen->trajBBGlobalPos, pGen->filterGlobalPos, sizeof(float)*3);
			memcpy(pGen->trajBBGlobalVel, pGen->filterGlobalVel, sizeof(float)*3);

			memcpy(pGen->trajGlobalPos, pGen->filterGlobalPos, sizeof(float)*3);
			memcpy(pGen->trajGlobalVel, pGen->filterGlobalVel, sizeof(float)*3);
			memset(pGen->trajGlobalAcc, 0, sizeof(float)*3);

			CtrlUtilTurnGlobal2Local(pGen->filterGlobalPos[2], pGen->filterGlobalVel[0], pGen->filterGlobalVel[1], pGen->trajLocalVel, pGen->trajLocalVel+1);
			pGen->trajLocalVel[2] = pGen->filterGlobalVel[2];
			memset(pGen->trajLocalAcc, 0, sizeof(float)*3);
		}
		break;
		case MODE_GLOBAL_POS:
		{
			switch(newMode)
			{
				case MODE_LOCAL_VEL:
				{
					CtrlUtilTurnGlobal2Local(pGen->trajGlobalPos[2], pGen->trajGlobalVel[0], pGen->trajGlobalVel[1], pGen->trajLocalVel, pGen->trajLocalVel+1);
					pGen->trajLocalVel[2] = pGen->trajGlobalVel[2];

					CtrlUtilTurnGlobal2Local(pGen->trajGlobalPos[2], pGen->trajGlobalAcc[0], pGen->trajGlobalAcc[1], pGen->trajLocalAcc, pGen->trajLocalAcc+1);
					pGen->trajLocalAcc[2] = pGen->trajGlobalAcc[2];
				}
				break;
			}
		}
		break;
		case MODE_LOCAL_VEL:
		{
			switch(newMode)
			{
				case MODE_GLOBAL_POS:
				{
					// position is not stored in vel mode, take it from filter
					memcpy(pGen->trajBBGlobalPos, pGen->filterGlobalPos, sizeof(float)*3);
					memcpy(pGen->trajGlobalPos, pGen->filterGlobalPos, sizeof(float)*3);

					CtrlUtilTurnLocal2Global(pGen->filterGlobalPos[2], pGen->trajLocalVel[0], pGen->trajLocalVel[1], pGen->trajGlobalVel, pGen->trajGlobalVel+1);
					pGen->trajGlobalVel[2] = pGen->trajLocalVel[2];
					memcpy(pGen->trajBBGlobalVel, pGen->trajGlobalVel, sizeof(float)*3);

					CtrlUtilTurnLocal2Global(pGen->filterGlobalPos[2], pGen->trajLocalAcc[0], pGen->trajLocalAcc[1], pGen->trajGlobalAcc, pGen->trajGlobalAcc+1);
					pGen->trajGlobalAcc[2] = pGen->trajLocalAcc[2];
				}
				break;
			}
		}
		break;
	}

	pGen->mode = newMode;
}

void TrajGeneratorInit(TrajGenerator* pGen)
{
	pGen->mode = MODE_RESET;

	pGen->dampingXY = 0.25f;
	pGen->dampingW = 0.05f;

	pGen->tShiftXY = 0.59f;
	pGen->tShiftW = 0.59f;

	pGen->vMaxXY = 2;
	pGen->aMaxXY = 3;
	pGen->jMaxXY = 5;
	pGen->vMaxW = 10;
	pGen->aMaxW = 20;
	pGen->jMaxW = 40;
}

void TrajGeneratorReset(TrajGenerator* pGen)
{
	pGen->mode = MODE_RESET;
}

void TrajGeneratorSetState(TrajGenerator* pGen, float* pGlobalPos, float* pGlobalVel)
{
	memcpy(pGen->filterGlobalPos, pGlobalPos, sizeof(float)*3);
	memcpy(pGen->filterGlobalVel, pGlobalVel, sizeof(float)*3);
}

void TrajGeneratorCreateGlobalPos(TrajGenerator* pGen, float* pPosTarget)
{
	changeMode(pGen, MODE_GLOBAL_POS);

	TrajSecOrder2DCreate(&pGen->trajPosXY, pGen->trajBBGlobalPos, pGen->trajBBGlobalVel, pPosTarget, pGen->vMaxXY, pGen->aMaxXY);

	float normTrgOrient = AngleNormalize(pPosTarget[2]);
	float orientB;
	if(normTrgOrient < 0)
		orientB = normTrgOrient+2*M_PI;
	else
		orientB = normTrgOrient-2*M_PI;

	TrajSecOrder1D trajB;

	TrajSecOrder1DCreate(&pGen->trajPosW, pGen->trajBBGlobalPos[2], pGen->trajBBGlobalVel[2], normTrgOrient, pGen->vMaxW, pGen->aMaxW);
	TrajSecOrder1DCreate(&trajB, pGen->trajBBGlobalPos[2], pGen->trajBBGlobalVel[2], orientB, pGen->vMaxW, pGen->aMaxW);

	if(TrajSecOrder1DGetTotalTime(&pGen->trajPosW) > TrajSecOrder1DGetTotalTime(&trajB))
		memcpy(&pGen->trajPosW, &trajB, sizeof(TrajSecOrder1D));
}

void TrajGeneratorCreateLocalVel(TrajGenerator* pGen, float* pVelTarget)
{
	changeMode(pGen, MODE_LOCAL_VEL);

	TrajSecOrder2DCreate(&pGen->trajVelXY, pGen->trajLocalVel, pGen->trajLocalAcc, pVelTarget, pGen->aMaxXY, pGen->jMaxXY);
	TrajSecOrder1DCreate(&pGen->trajVelW, pGen->trajLocalVel[2], pGen->trajLocalAcc[2], pVelTarget[2], pGen->aMaxW, pGen->jMaxW);
}

void TrajGeneratorCreateGlobalVelAndOrient(TrajGenerator* pGen, float* pVelXY, float orient)
{
	changeMode(pGen, MODE_GLOBAL_VEL_AND_ORIENT);

	TrajSecOrder2DCreate(&pGen->trajVelXY, pGen->trajGlobalVel, pGen->trajGlobalAcc, pVelXY, pGen->aMaxXY, pGen->jMaxXY);

	float normTrgOrient = AngleNormalize(orient);
	float orientB;
	if(normTrgOrient < 0)
		orientB = normTrgOrient+2*M_PI;
	else
		orientB = normTrgOrient-2*M_PI;

	TrajSecOrder1D trajB;

	TrajSecOrder1DCreate(&pGen->trajPosW, pGen->trajBBGlobalPos[2], pGen->trajBBGlobalVel[2], normTrgOrient, pGen->vMaxW, pGen->aMaxW);
	TrajSecOrder1DCreate(&trajB, pGen->trajBBGlobalPos[2], pGen->trajBBGlobalVel[2], orientB, pGen->vMaxW, pGen->aMaxW);

	if(TrajSecOrder1DGetTotalTime(&pGen->trajPosW) > TrajSecOrder1DGetTotalTime(&trajB))
		memcpy(&pGen->trajPosW, &trajB, sizeof(TrajSecOrder1D));
}

void TrajGeneratorStep(TrajGenerator* pGen, float tStep)
{
	switch(pGen->mode)
	{
		case MODE_GLOBAL_POS:
		{
			const float dXY = pGen->dampingXY;
			const float dW = pGen->dampingW;
			const float tShiftXY = pGen->tShiftXY;
			const float tShiftW = pGen->tShiftW;

			float fXY[3];
			float fW[3];

			fXY[0] = 60.0f/(dXY*dXY*dXY);
			fXY[1] = -36.0f/(dXY*dXY);
			fXY[2] = -9.0f/dXY;

			fW[0] = 60.0f/(dW*dW*dW);
			fW[1] = -36.0f/(dW*dW);
			fW[2] = -9.0f/dW;

			float posTrg[3];

			TrajSecOrder2DValuesAtTime(&pGen->trajPosXY, dXY*tShiftXY, posTrg, 0, 0);
			TrajSecOrder1DValuesAtTime(&pGen->trajPosW, dW*tShiftW, posTrg+2, 0, 0);

			float j[3];
			j[0] = fXY[0]*(posTrg[0]-pGen->trajGlobalPos[0]) + fXY[1]*pGen->trajGlobalVel[0] + fXY[2]*pGen->trajGlobalAcc[0];
			j[1] = fXY[0]*(posTrg[1]-pGen->trajGlobalPos[1]) + fXY[1]*pGen->trajGlobalVel[1] + fXY[2]*pGen->trajGlobalAcc[1];
			j[2] = fW[0]*AngleNormalize(posTrg[2]-pGen->trajGlobalPos[2]) + fW[1]*pGen->trajGlobalVel[2] + fW[2]*pGen->trajGlobalAcc[2];

			uint8_t i = 0;
			for(; i < 3; i++)
			{
				pGen->trajGlobalPos[i] += tStep*(pGen->trajGlobalVel[i] + tStep*(0.5f*pGen->trajGlobalAcc[i] + 1.0f/6.0f*j[i]*tStep));
				pGen->trajGlobalVel[i] += tStep*(pGen->trajGlobalAcc[i] + 0.5f*j[i]*tStep);
				pGen->trajGlobalAcc[i] += j[i]*tStep;
				pGen->trajGlobalJerk[i] = j[i];
			}

			TrajSecOrder2DValuesAtTime(&pGen->trajPosXY, tStep, pGen->trajBBGlobalPos, pGen->trajBBGlobalVel, 0);
			TrajSecOrder1DValuesAtTime(&pGen->trajPosW, tStep, pGen->trajBBGlobalPos+2, pGen->trajBBGlobalVel+2, 0);

			AngleNormalizePtr(&pGen->trajGlobalPos[2]);
			AngleNormalizePtr(&pGen->trajBBGlobalPos[2]);
		}
		break;
		case MODE_LOCAL_VEL:
		{
			TrajSecOrder2DValuesAtTime(&pGen->trajVelXY, tStep, pGen->trajLocalVel, pGen->trajLocalAcc, pGen->trajLocalJerk);
			TrajSecOrder1DValuesAtTime(&pGen->trajVelW, tStep, pGen->trajLocalVel+2, pGen->trajLocalAcc+2, pGen->trajLocalJerk+2);
		}
		break;
		case MODE_GLOBAL_VEL_AND_ORIENT:
		{
			TrajSecOrder2DValuesAtTime(&pGen->trajVelXY, tStep, pGen->trajGlobalVel, pGen->trajGlobalAcc, 0);

//			TrajMinJerk1DBBValuesAtTime(&pGen->trajPosW, tStep, pGen->trajBBGlobalPos+2, pGen->trajBBGlobalVel+2, 0);
//			TrajMinJerk1DValuesAtTime(&pGen->trajPosW, tStep, pGen->trajGlobalPos+2, pGen->trajGlobalVel+2, pGen->trajGlobalAcc+2, 0);
		}
		break;
	}
}
