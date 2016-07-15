/*
 * DelayEstimator.cpp
 *
 * Correlation calculation is based on http://paulbourke.net/miscellaneous/correlate/
 *
 *  Created on: Jun 18, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/util/DelayEstimator.h"

#include <cmath>
#include <iostream>

namespace affw {

DelayEstimator::DelayEstimator(double timeHorizon, double precision, double minDelay, double maxDelay) :
		x_buffer(timeHorizon / precision),
		y_buffer(timeHorizon / precision)
{
	this->timeHorizon = timeHorizon;
	this->precision = precision;
	this->minDelay = minDelay;
	this->maxDelay = maxDelay;
	this->lastTime = -1;
	this->delay = 0;

	int bufferSize = timeHorizon / precision;

}

DelayEstimator::~DelayEstimator() {
}

void DelayEstimator::update(double time, double x, double y)
{
	if(lastTime > 0 && !x_buffer.empty() && ! y_buffer.empty())
	{
		double px = x_buffer[x_buffer.size()-1];
		double py = y_buffer[y_buffer.size()-1];
		double dx = x - px;
		double dy = y - py;
		double bt = lastTime;

		for(double t = bt+this->precision; t<=time; t+=this->precision)
		{
			double rel = (time - t) / (time - bt);
			x_buffer.push_back(px + rel * dx);
			y_buffer.push_back(py + rel * dy);
		}
		lastTime = time;
	} else {
		x_buffer.push_back(x);
		y_buffer.push_back(y);
		lastTime = time;
	}

	double delay;
	bool valid = delay_by_cross_correlation(delay);
	if(valid)
	{
		this->delay = delay;
	}
}

bool DelayEstimator::delay_by_cross_correlation(double& delay)
{
	double mu_x = 0, mu_y = 0;
	for(int i=0;i<x_buffer.size();i++) mu_x += x_buffer[i];
	for(int i=0;i<y_buffer.size();i++) mu_y += y_buffer[i];
	mu_x /= x_buffer.size();
	mu_y /= y_buffer.size();

	double sig_x = 0; double sig_y = 0;
	for(int i=0; i<x_buffer.size(); i++)
		sig_x += (x_buffer[i] - mu_x) * (x_buffer[i] - mu_x);
	for(int i=0; i<y_buffer.size(); i++)
			sig_y += (y_buffer[i] - mu_y) * (y_buffer[i] - mu_y);

	double denominator = std::sqrt(sig_x * sig_y);
	if(denominator <= 0 || !std::isfinite(denominator))
	{
		return false;
	}

	double max_corr = 0;
	int max_d = 0;
	for(int d = this->minDelay/precision; d <= this->maxDelay/precision; d++)
	{
		double sxy = 0;
		for(int i=0;i<x_buffer.size(); i++)
		{
			double ii = i + d;
			if(ii < 0 || ii >= x_buffer.size())
				continue;
			sxy += (x_buffer[ii] - mu_x) * (y_buffer[ii] - mu_y);
		}
		double r = sxy / denominator;
		if(r > max_corr)
		{
			max_corr = r;
			max_d = d;
		}
	}

	delay = max_d * precision;
	return true;
}

} /* namespace affw */
