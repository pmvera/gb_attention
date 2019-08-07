#include <ros/ros.h>

#include <gb_attention_msgs/AttentionPoints.h>
#include <gb_attention_msgs/RemoveAttentionStimuli.h>

#include "gb_attention/stimuli/TableAttentionClient.h"

#include <list>

namespace gb_attention
{

TableAttentionClient::TableAttentionClient()
: AttentionClient("table")
{
}

std::list<geometry_msgs::Point>
TableAttentionClient::get_attention_points(const bica_graph::StringEdge& edge)
{
  std::list<geometry_msgs::Point> ret;
  const tf::Transform trans = graph_.get_tf_edge(edge.get_source(), edge.get_target()).get();

  geometry_msgs::Point p1;

  p1.x = trans.getOrigin().x();
  p1.y = trans.getOrigin().y();
  p1.z = trans.getOrigin().z();

  ret.push_back(p1);

  return ret;
}

};  // namespace gb_attention
