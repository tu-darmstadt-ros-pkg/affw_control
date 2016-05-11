/*
 * joy2vel.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Header.h>
#include <cmath>
#include <string>
#include <assert.h>

geometry_msgs::Twist targetVel;

double vDef = 1;
double vMax = 3;
double dccDef = 2;
double dccMax = 6;
double accDef = 2;
double accMax = 4;
double rotateDef = 5;
double rotateMax = 10;
double rotateMin = 1;

double uDcc = 0;
double uAcc = 0;

void callbackJoy(const sensor_msgs::Joy::ConstPtr& joy) {

	uAcc = -(joy->axes[5]-1)/2;
	uDcc = -(joy->axes[2]-1)/2;

	double vmax = vDef + (vMax-vDef) * uAcc;
	double rotmax = rotateDef + (rotateMax-rotateDef) * uAcc;
	geometry_msgs::Twist vel;
	vel.linear.x = joy->axes[1] * vmax;
	vel.linear.y = joy->axes[0] * vmax;
	vel.angular.z = joy->axes[3] * rotmax;
	targetVel = vel;
}

double angleBetweenVectorAndVector(double v1x, double v1y, double v2x, double v2y)
{
	// angle between positive x-axis and first vector
	double angleA = atan2(v1x, v1y);
	// angle between positive x-axis and second vector
	double angleB = atan2(v2x, v2y);
	// rotation
	double rotation = angleB - angleA;
	// fix overflows
	if (rotation < (-M_PI - 0.001))
	{
		rotation += 2 * M_PI;
	} else if (rotation > (M_PI + 0.001))
	{
		rotation -= 2 * M_PI;
	}
	return fabsf(rotation);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_joy");
	ros::NodeHandle n;

	bool affw = false;
	ros::param::param<double>("vDef", vDef, vDef);
	ros::param::param<double>("vMax", vMax, vMax);
	ros::param::param<double>("dccDef", dccDef, dccDef);
	ros::param::param<double>("dccMax", dccMax, dccMax);
	ros::param::param<double>("accDef", accDef, accDef);
	ros::param::param<double>("accMax", accMax, accMax);
	ros::param::param<double>("rotateDef", rotateDef, rotateDef);
	ros::param::param<double>("rotateMax", rotateMax, rotateMax);
	ros::param::param<double>("rotateMin", rotateMin, rotateMin);
	ros::param::param<bool>("affw", affw, affw);

	// affw enabled?
	if(affw)
		ROS_INFO("affw enabled");
	else
		ROS_INFO("affw disabled");

	std::string topic;
	if(affw)
		topic = "/affw_ctrl/target_vel";
	else
		topic = "/cmd_vel";

	ros::Publisher pub_vel;
	if(affw)
		pub_vel = n.advertise<geometry_msgs::TwistStamped>(topic, 1);
	else
		pub_vel = n.advertise<geometry_msgs::Twist>(topic, 1);

	ros::Subscriber sub_joy = n.subscribe("/joy", 1,
			callbackJoy);

	geometry_msgs::Twist curVel;
	double dt = 0.01;
	int seq = 0;
	while(ros::ok())
	{
		double acc;
		if (targetVel.linear.x == 0 && targetVel.linear.y == 0)
		{
			// dcc
			acc = (dccDef + ((dccMax - dccDef) * uDcc));
		} else if ((curVel.linear.x == 0 && curVel.linear.y == 0)
				|| (angleBetweenVectorAndVector(targetVel.linear.x, targetVel.linear.y, curVel.linear.x, curVel.linear.y) < M_PI/2))
		{
			// acc
			acc = (accDef + ((accMax - accDef) * uAcc));
		} else
		{
			// dcc
			acc = (dccDef + ((dccMax - dccDef) * uDcc));
		}

		double dx = targetVel.linear.x - curVel.linear.x;
		double dy = targetVel.linear.y - curVel.linear.y;
		double ldxy = sqrt(dx*dx+dy*dy);
		if (ldxy > (acc * dt))
		{
			curVel.linear.x = curVel.linear.x + (dx / ldxy) * (acc * dt);
			curVel.linear.y = curVel.linear.y + (dy / ldxy) * (acc * dt);
		} else
		{
			curVel.linear.x = targetVel.linear.x;
			curVel.linear.y = targetVel.linear.y;
		}

		curVel.angular.z = targetVel.angular.z;

		if(affw)
		{
			geometry_msgs::TwistStamped curVelStamp;
			curVelStamp.header.frame_id = "base_link";
			curVelStamp.header.seq = seq++;
			curVelStamp.header.stamp = ros::Time::now();
			curVelStamp.twist = curVel;
			pub_vel.publish(curVelStamp);
		} else
			pub_vel.publish(curVel);
		ros::spinOnce();

		ros::Duration(dt).sleep();
	}

	return 0;
}


