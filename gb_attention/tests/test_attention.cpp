#include <iostream>
#include <string>

#include "gtest/gtest.h"

#include <bica_graph/graph_client.h>
#include <bica_graph/graph_server.h>

TEST(ClaseTest, methods)
{
  ros::Time::init();

  ros::Rate rate(1);
  ros::Time start = ros::Time::now();

  auto graph_client = std::make_shared<bica_graph::GraphClient>();

  graph_client->add_node("leia", "robot");
  graph_client->add_node("mesa_1", "table");
  graph_client->add_node("mesa_2", "table");
  graph_client->add_node("Paco", "person");

  start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 1.0 )
  {
    ros::spinOnce();
    rate.sleep();
  }
  graph_client->set_tf_identity("base_footprint", "leia");

  graph_client->add_edge("leia", "want_see", "mesa_1");
  graph_client->add_edge("leia", "want_see", "mesa_2");
  //graph_client->add_edge("leia", "want_see", "Paco");

  tf::Transform tf_r2t(tf::Quaternion(0, 0, 0, 1), tf::Vector3(3, 0, 0));
  graph_client->add_edge("leia", tf_r2t, "mesa_1", true);
  tf::Transform tf_r2t2(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2, 0, 0));
  graph_client->add_edge("leia", tf_r2t2, "mesa_2", true);
  tf::Transform tf_r2p(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 0, 0));
  graph_client->add_edge("leia", tf_r2t, "Paco", true);

  start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 15.0 )
  {
    ros::spinOnce();
    rate.sleep();
  }

  graph_client->remove_edge("leia", "want_see", "mesa_2");

  start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 15.0 )
  {
    ros::spinOnce();
    rate.sleep();
  }

}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_attention");
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
