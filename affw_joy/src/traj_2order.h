/*
 * traj_2order.h
 *
 *  Created on: 21.05.2016
 *      Author: AndreR
 */

#ifndef TRAJ_2ORDER_H_
#define TRAJ_2ORDER_H_

#include <stdint.h>

#define TRAJ_2ORDER_PARTS 3

typedef struct _TrajSecOrderPart
{
	float tEnd;

	// xdd(t) = xdd
	// xd(t) = xd0 + xdd*t
	// x(t) = x0 + xd0*t + 0.5*xdd*t^2
	float xdd; 	// x dot dot, control signal
	float xd0;	// x dot initial value
	float x0;	// x initial value
} TrajSecOrderPart;

typedef struct _TrajSecOrder1D
{
	TrajSecOrderPart parts[TRAJ_2ORDER_PARTS];
} TrajSecOrder1D;

typedef struct _TrajSecOrder2D
{
	TrajSecOrderPart x[TRAJ_2ORDER_PARTS];
	TrajSecOrderPart y[TRAJ_2ORDER_PARTS];
} TrajSecOrder2D;

void	TrajSecOrder1DCreate(TrajSecOrder1D* pTraj, float x0, float xd0, float xTrg, float xdMax, float xddMax);
void	TrajSecOrder1DValuesAtTime(const TrajSecOrder1D* pTraj, float t, float* pX, float* pXd, float* pXdd);
float	TrajSecOrder1DGetTotalTime(const TrajSecOrder1D* pTraj);
float	TrajSecOrder1DGetFinalX(const TrajSecOrder1D* pTraj);

void	TrajSecOrder2DCreate(TrajSecOrder2D* pTraj, float* pX0, float* pXd0, float* pXTrg, float xdMax, float xddMax);
void	TrajSecOrder2DValuesAtTime(const TrajSecOrder2D* pTraj, float t, float* pX, float* pXd, float* pXdd);
float	TrajSecOrder2DGetTotalTime(const TrajSecOrder2D* pTraj);

#endif /* TRAJ_2ORDER_H_ */
