
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_datatypes.h>

#include <string>
#include <iostream>
#include <math.h>

#include <boost/circular_buffer.hpp>

ros::Publisher pub_state;
const int BUFFER_SIZE = 1;
boost::circular_buffer<geometry_msgs::PoseStamped> poseBuffer(BUFFER_SIZE);
geometry_msgs::PoseStamped lastPose;

boost::circular_buffer<double> vel_x_buffer(BUFFER_SIZE);
boost::circular_buffer<double> vel_y_buffer(BUFFER_SIZE);
boost::circular_buffer<double> vel_w_buffer(BUFFER_SIZE);

double f[BUFFER_SIZE];

double getOrientation(geometry_msgs::Quaternion gq) {
	double ow = gq.w;
	double ox = gq.x;
	double oy = gq.y;
	double oz = gq.z;

	tf::Quaternion q(ox, oy, oz, ow);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}

double normalizeAngle(double angle)
{
	// Don't call this a hack! It's numeric!
	return (angle - (round((angle / (M_PI*2)) - 1e-6) * M_PI*2));
}

void createGaussFilter(int filterSize, double* f)
{
	const double sig = 2;
	const double sig_sq = sig*sig;
	for(int i=0;i<filterSize;i++)
	{
		double xx = i - (int)(filterSize/2);

		f[i] = exp((-1.0) * (xx*xx) / (2*sig_sq)) / sqrt(2*M_PI*sig_sq);
	}
	double sum = 0;
	for(int i=0;i<filterSize;i++)
	{
		sum += f[i];
	}
	for(int i=0;i<filterSize;i++)
	{
		f[i] /= sum;
	}
}

double applyGaussFilter(boost::circular_buffer<double> x, double* f)
{
	int filterSize = x.size();
	double result = 0;
	for(int i=0;i<filterSize;i++)
	{
		result += f[i] * x[i];
	}
	return result;
}

double determineContinuousAngle(double oldAngle, double newAngle)
{
	// project new orientation next to old one
	// thus values greater than PI are possible but the handling gets easier
	// standard: just a small rotation
	double dAngle = newAngle - normalizeAngle(oldAngle);
	// rotation clockwise over Pi-border
	if (dAngle > M_PI)
	{
		dAngle = dAngle - (2 * M_PI);
	}
	// rotation counter-clockwise over Pi-border
	else if (dAngle < -M_PI)
	{
		dAngle = dAngle + (2 * M_PI);
	}
	return oldAngle + dAngle;
}

void smoothPose(geometry_msgs::PoseStamped& pose)
{
	double sum[3];
	double firstOri = getOrientation(poseBuffer[0].pose.orientation);
	for(int i=0;i<poseBuffer.size();i++)
	{
		sum[0] += poseBuffer[i].pose.position.x;
		sum[1] += poseBuffer[i].pose.position.y;
		double ori = getOrientation(poseBuffer[i].pose.orientation);

		sum[2] += determineContinuousAngle(firstOri, ori);
	}
	pose.pose.position.x = sum[0] / poseBuffer.size();
	pose.pose.position.y = sum[1] / poseBuffer.size();
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(sum[2] / poseBuffer.size());
}


void stateCallback(const nav_msgs::Odometry::ConstPtr& state)
{
	geometry_msgs::PoseStamped newPose;
	newPose.pose = state->pose.pose;
	newPose.header = state->header;
	poseBuffer.push_back(newPose);

	if(poseBuffer.size() == poseBuffer.capacity())
	{
		geometry_msgs::PoseStamped smoothedPose;
		smoothedPose.header.frame_id = newPose.header.frame_id;
		smoothedPose.header.stamp = poseBuffer[poseBuffer.size()-1].header.stamp;
		smoothPose(smoothedPose);

		if(lastPose.header.stamp.isValid())
		{
			double dt = (smoothedPose.header.stamp - lastPose.header.stamp).toSec();
			if(dt > 0)
			{
				geometry_msgs::TwistStamped twist;
				twist.header = state->header;

				twist.twist.linear.x = (smoothedPose.pose.position.x - lastPose.pose.position.x) / dt;
				twist.twist.linear.y = (smoothedPose.pose.position.y - lastPose.pose.position.y) / dt;
				double lastOri = getOrientation(lastPose.pose.orientation);
				double newOri = determineContinuousAngle(lastOri, getOrientation(smoothedPose.pose.orientation));
				twist.twist.angular.z = normalizeAngle(newOri - lastOri) / dt;

				vel_x_buffer.push_back(twist.twist.linear.x);
				vel_y_buffer.push_back(twist.twist.linear.y);
				vel_w_buffer.push_back(twist.twist.angular.z);
				if(vel_x_buffer.size() == vel_x_buffer.capacity())
				{
					twist.twist.linear.x = applyGaussFilter(vel_x_buffer, f);
					twist.twist.linear.y = applyGaussFilter(vel_y_buffer, f);
					twist.twist.angular.z = applyGaussFilter(vel_w_buffer, f);

					nav_msgs::Odometry odom;
					odom.header = smoothedPose.header;
					odom.pose.pose = smoothedPose.pose;
					odom.twist.twist = twist.twist;
					pub_state.publish(odom);
				}
			}
		}

		lastPose = smoothedPose;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "affw_pos2vel");
	ros::NodeHandle n;

	std::string state_topic;
	if(!ros::param::get("state_topic", state_topic))
	{
		return 100;
	}

	createGaussFilter(vel_x_buffer.capacity(), f);
	for(int i=0;i<BUFFER_SIZE;i++)
		std::cout << f[i] << std::endl;

	pub_state = n.advertise<nav_msgs::Odometry>("/state", 1);
	ros::TransportHints th;
	th.unreliable();
	ros::Subscriber sub_state = n.subscribe(state_topic, 1, stateCallback, th);

	ros::spin();

	return 0;
}
