#include <ros/ros.h>

#include <bica_graph/graph_client.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_attention_node");
  ros::NodeHandle n;

  ros::Rate rate(1);
  ros::Time start = ros::Time::now();

  bica_graph::GraphClient graph_client;

  ROS_INFO("Creating nodes...");
  graph_client.add_node("leia", "robot");
  graph_client.add_node("mesa_1", "table");
  graph_client.add_node("mesa_2", "table");
  graph_client.add_node("Paco", "person");

  ROS_INFO("done");

  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 1.0 )
  {
    ROS_INFO("spinnging 0");
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Creating identify");
  graph_client.set_tf_identity("base_footprint", "leia");

  ROS_INFO("Creating static transforms");


  ROS_INFO("wanting see");
  graph_client.add_edge("leia", "want_see", "mesa_1");
  graph_client.add_edge("leia", "want_see", "mesa_2");
  //graph_client.add_edge("leia", "want_see", "Paco");



  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 20.0 )
  {
    tf2::Transform tf_r2t(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(3, 3, 0));
    graph_client.add_edge("leia", tf_r2t, "mesa_1");
    tf2::Transform tf_r2t2(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(2, -2, 0));
    graph_client.add_edge("leia", tf_r2t2, "mesa_2");
    tf2::Transform tf_r2p(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(1, 0, 1));
    graph_client.add_edge("leia", tf_r2t, "Paco");


    ROS_INFO("spinnging 1");
    ros::spinOnce();
    rate.sleep();
  }
    ROS_INFO("removing want see 2");
  graph_client.remove_edge("leia", "want_see", "mesa_2");

  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 15.0 )
  {
    ROS_INFO("spinnging 2");
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
