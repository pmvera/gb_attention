#ifndef GB_ATTENTION_ATTENTIONCLIENT_H
#define GB_ATTENTION_ATTENTIONCLIENT_H

#include <ros/ros.h>

#include <bica_graph/graph_client.h>

namespace gb_attention
{

class AttentionClient
{
public:
	AttentionClient(const std::string& class_id);

	void update();

	virtual std::list<geometry_msgs::PointStamped> get_attention_points(const bica_graph::StringEdge& edge) = 0;
protected:
	ros::NodeHandle nh_;
	bica_graph::GraphClient graph_;

  std::string working_frame_;
  ros::Publisher attention_points_pub_;
	ros::ServiceClient remove_instance_service_;
	std::string class_;

	std::set<std::string> instances_sent_;
};

};  // namespace gb_attention

#endif  // GB_ATTENTION_ATTENTIONCLIENT_H
