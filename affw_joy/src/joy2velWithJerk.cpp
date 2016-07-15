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
#include <signal.h>

ros::Publisher pub_vel_cmd;
geometry_msgs::Twist targetVel;
bool running = true;

double vDef = 1;
double vMax = 3;
double dccDef = 2;
double dccMax = 6;
double accDef = 2;
double accMax = 4;
double jerkDef = 10;
double rotateDef = 5;
double rotateMax = 10;
double rotateMin = 1;
double accW_ = 25;

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

void mySigintHandler(int sig)
{
	running = false;

	geometry_msgs::Twist zeroVel;
	pub_vel_cmd.publish(zeroVel);
	ros::spinOnce();

	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}

void step(geometry_msgs::Twist& cur, geometry_msgs::Twist& target, double dTarget, double dt)
{
	double dx = target.linear.x - cur.linear.x;
	double dy = target.linear.y - cur.linear.y;
	double ldxy = sqrt(dx*dx+dy*dy);
	if (ldxy > (dTarget * dt))
	{
		cur.linear.x = cur.linear.x + (dx / ldxy) * (dTarget * dt);
		cur.linear.y = cur.linear.y + (dy / ldxy) * (dTarget * dt);
	} else
	{
		cur.linear.x = target.linear.x;
		cur.linear.y = target.linear.y;
	}
}

double length(const geometry_msgs::Twist& v)
{
	return sqrt(v.linear.x*v.linear.x+v.linear.y*v.linear.y);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_joy");
	ros::NodeHandle n;

	bool affw = false;
	std::string cmd_vel_topic = "cmd_vel";
	ros::param::param<double>("vDef", vDef, vDef);
	ros::param::param<double>("vMax", vMax, vMax);
	ros::param::param<double>("dccDef", dccDef, dccDef);
	ros::param::param<double>("dccMax", dccMax, dccMax);
	ros::param::param<double>("accDef", accDef, accDef);
	ros::param::param<double>("accMax", accMax, accMax);
	ros::param::param<double>("jerkDef", jerkDef, jerkDef);
	ros::param::param<double>("rotateDef", rotateDef, rotateDef);
	ros::param::param<double>("rotateMax", rotateMax, rotateMax);
	ros::param::param<double>("rotateMin", rotateMin, rotateMin);
	ros::param::param<double>("rotateAcc", accW_, accW_);
	ros::param::param<bool>("affw", affw, affw);
	ros::param::get("cmd_vel_topic", cmd_vel_topic);

	// affw enabled?
	if(affw)
		ROS_INFO("affw enabled");
	else
		ROS_INFO("affw disabled");

	pub_vel_cmd = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
	ros::Publisher pub_vel_target = n.advertise<geometry_msgs::TwistStamped>("/affw_ctrl/target_vel", 1);
	ros::Subscriber sub_joy = n.subscribe("/joy", 1, callbackJoy);

	signal(SIGINT, mySigintHandler);

	geometry_msgs::Twist curVel;
	geometry_msgs::Twist curAcc;
	geometry_msgs::Twist targetAcc;
	double dt = 0.01;
	int seq = 0;
	while(ros::ok() && running)
	{
		double curVelLen = length(curVel);
		double targetVelLen = length(targetVel);
		double acc;
		if (targetVel.linear.x == 0 && targetVel.linear.y == 0)
		{
			// dcc
			acc = (dccDef + ((dccMax - dccDef) * uDcc));
			if(curVelLen > 0.0001)
			{
				targetAcc.linear.x = curVel.linear.x  / curVelLen * -acc;
				targetAcc.linear.y = curVel.linear.y  / curVelLen * -acc;
			} else {
				targetAcc.linear.x = 0;
				targetAcc.linear.y = 0;
			}
		} else if ((curVel.linear.x == 0 && curVel.linear.y == 0)
				|| (angleBetweenVectorAndVector(targetVel.linear.x, targetVel.linear.y, curVel.linear.x, curVel.linear.y) < M_PI/2))
		{
			// acc
			acc = (accDef + ((accMax - accDef) * uAcc));
			targetAcc.linear.x = targetVel.linear.x  / targetVelLen * acc;
			targetAcc.linear.y = targetVel.linear.y  / targetVelLen * acc;
		} else
		{
			// dcc
			acc = (dccDef + ((dccMax - dccDef) * uDcc));
			double dx = targetVel.linear.x - curVel.linear.x;
			double dy = targetVel.linear.y - curVel.linear.y;
			double ldxy = sqrt(dx*dx+dy*dy);
			targetAcc.linear.x = dx / ldxy * -acc;
			targetAcc.linear.y = dy / ldxy * -acc;
		}

		step(curAcc, targetAcc, jerkDef, dt);
		double curAccLen = length(curAcc);
		step(curVel, targetVel, curAccLen, dt);

		double dw = targetVel.angular.z - curVel.angular.z;
		double accW = accW_;
		if(dw < 0) accW *= -1;
		if(fabsf(dw) > (fabsf(accW) * dt))
		{
			curVel.angular.z = curVel.angular.z + accW * dt;
		} else
		{
			curVel.angular.z = targetVel.angular.z;
		}

		if(affw)
		{
			geometry_msgs::TwistStamped curVelStamp;
			curVelStamp.header.frame_id = "base_link";
			curVelStamp.header.seq = seq++;
			curVelStamp.header.stamp = ros::Time::now();
			curVelStamp.twist = curVel;
			pub_vel_target.publish(curVelStamp);
		} else
			pub_vel_cmd.publish(curVel);
		ros::spinOnce();

		ros::Duration(dt).sleep();
	}

	mySigintHandler(0);

	return 0;
}


