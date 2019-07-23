#ifndef CHANGE_H
#define CHANGE_H

#include "ros/ros.h"
#include "std_msgs/Int64.h"

class Clase
{
public:
	Clase();

	void messageCallback(const std_msgs::Int64::ConstPtr& msg);

  int duplicate(int value);
private:
	ros::NodeHandle n_;
	ros::Subscriber sub_;
};

#endif
