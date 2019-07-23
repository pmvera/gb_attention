#include "ros/ros.h"
#include "gb_attention/Class.h"


int main(int argc, char **argv)
{

   ros::init(argc, argv, "num_subscriber");
   ros::NodeHandle n;

   Clase Clase;

   ros::spin();

   return 0;

 }
