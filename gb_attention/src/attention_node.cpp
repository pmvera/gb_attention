#include "ros/ros.h"
#include "gb_attention/AttentionServer.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "attention_node");
  ros::NodeHandle n;

  gb_attention::AttentionServer attention_server;

  ros::Rate rate(1);

  while(ros::ok())
  {
    attention_server.update();

    ros::spinOnce();
    rate.sleep();
  }


   return 0;

 }
