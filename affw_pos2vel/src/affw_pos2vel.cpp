
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_datatypes.h>

#include <string>
#include <iostream>
#include <math.h>

#include <boost/circular_buffer.hpp>

ros::Publisher pub_state;
const int BUFFER_SIZE = 7;
boost::circular_buffer<geometry_msgs::PoseStamped> poseBuffer(BUFFER_SIZE);

boost::circular_buffer<double> vel_x_buffer(BUFFER_SIZE);
boost::circular_buffer<double> vel_y_buffer(BUFFER_SIZE);
boost::circular_buffer<double> vel_w_buffer(BUFFER_SIZE);

double f[BUFFER_SIZE];

bool differential = true;

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

void stateCallback(const geometry_msgs::PoseStamped::ConstPtr& state)
{
	geometry_msgs::PoseStamped newPose;
	newPose.pose = state->pose;
	newPose.header = state->header;

	if(poseBuffer.size() > 0)
	{
		const geometry_msgs::PoseStamped lastPose = poseBuffer[poseBuffer.size()-1];
		double dt = (newPose.header.stamp - lastPose.header.stamp).toSec();

		geometry_msgs::TwistStamped twist;
		twist.header = state->header;

		twist.twist.linear.x = (newPose.pose.position.x - lastPose.pose.position.x) / dt;
		twist.twist.linear.y = (newPose.pose.position.y - lastPose.pose.position.y) / dt;
		twist.twist.angular.z = normalizeAngle(getOrientation(newPose.pose.orientation) - getOrientation(lastPose.pose.orientation)) / dt;

		vel_x_buffer.push_back(twist.twist.linear.x);
		vel_y_buffer.push_back(twist.twist.linear.y);
		vel_w_buffer.push_back(twist.twist.angular.z);
		poseBuffer.push_back(newPose);
		if(vel_x_buffer.size() == vel_x_buffer.capacity())
		{
			twist.twist.linear.x = applyGaussFilter(vel_x_buffer, f);
			twist.twist.linear.y = applyGaussFilter(vel_y_buffer, f);
			if(differential)
			{
				twist.twist.linear.x = sqrt(twist.twist.linear.x*twist.twist.linear.x+
						twist.twist.linear.y*twist.twist.linear.y);
				twist.twist.linear.y = 0;
			}
			twist.twist.angular.z = applyGaussFilter(vel_w_buffer, f);

			nav_msgs::Odometry odom;
			odom.header = poseBuffer[poseBuffer.size()/2].header;
			odom.pose.pose = poseBuffer[poseBuffer.size()/2].pose;
			odom.twist.twist = twist.twist;
			pub_state.publish(odom);
		}
	} else {
		poseBuffer.push_back(newPose);
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "affw_pos2vel");
	ros::NodeHandle n;

	std::string pose_topic;
	if(!ros::param::get("pose_topic", pose_topic))
	{
		return 100;
	}

	std::string state_topic = "/state";
	ros::param::get("state_topic", state_topic);

	createGaussFilter(vel_x_buffer.capacity(), f);
	for(int i=0;i<BUFFER_SIZE;i++)
		std::cout << f[i] << std::endl;

	pub_state = n.advertise<nav_msgs::Odometry>(state_topic, 1);
	ros::TransportHints th;
	th.unreliable();
	ros::Subscriber sub_state = n.subscribe(pose_topic, 1, stateCallback, th);

	ros::spin();

	return 0;
}
