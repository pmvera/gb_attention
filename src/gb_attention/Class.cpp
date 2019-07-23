#include "ros/ros.h"

#include "gb_attention/Class.h"

Clase::Clase()
: n_()
{
	sub_ = n_.subscribe("message", 1, &Clase::messageCallback, this);
}

void
Clase::messageCallback(const std_msgs::Int64::ConstPtr& msg)
{
	ROS_INFO("Message: [%ld]", msg->data);
}

int
Clase::duplicate(int value)
{
  return value * 2;
}
