#include "ros/ros.h"
#include "gb_attention/Attention.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "attention_node");
  ros::NodeHandle n;

  Attention attention;

  ros::Rate rate(1);

  while(ros::ok())
  {
    attention.update();

    ros::spinOnce();
    rate.sleep();
  }


   return 0;

 }
