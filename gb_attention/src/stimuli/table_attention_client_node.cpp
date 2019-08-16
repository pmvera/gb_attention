#include <ros/ros.h>

#include <gb_attention_msgs/AttentionPoints.h>
#include <gb_attention_msgs/RemoveAttentionStimuli.h>

#include "gb_attention/AttentionClient.h"
#include <list>

class TableAttentionClient: public gb_attention::AttentionClient
{
public:
	TableAttentionClient()
  : AttentionClient("table")
  {
  }

	std::list<geometry_msgs::PointStamped> get_attention_points(const bica_graph::StringEdge& edge)
  {
    std::list<geometry_msgs::PointStamped> ret;
    const tf2::Transform trans = graph_.get_tf_edge(edge.get_source(), edge.get_target()).get();

    geometry_msgs::PointStamped p1;

		p1.header.stamp = ros::Time::now();
		p1.header.frame_id = edge.get_source();
    p1.point.x = trans.getOrigin().x();
    p1.point.y = trans.getOrigin().y();
    p1.point.z = trans.getOrigin().z();

    ret.push_back(p1);

    return ret;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_attention_client_node");
  ros::NodeHandle n;

  TableAttentionClient table_attention_client;

  ros::Rate rate(1);

  while(ros::ok())
  {
    table_attention_client.update();

    ros::spinOnce();
    rate.sleep();
  }

   return 0;
 }
