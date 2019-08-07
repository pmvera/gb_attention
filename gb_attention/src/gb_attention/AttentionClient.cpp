#include <ros/ros.h>

#include <gb_attention_msgs/AttentionPoints.h>
#include <gb_attention_msgs/RemoveAttentionStimuli.h>

#include "gb_attention/AttentionClient.h"

namespace gb_attention
{

AttentionClient::AttentionClient(const std::string& class_id)
: nh_(),
	class_(class_id)
{
	attention_points_pub_ = nh_.advertise<gb_attention_msgs::AttentionPoints>(
		"/attention/attention_points", 100);

	remove_instance_service_ = nh_.serviceClient<gb_attention_msgs::RemoveAttentionStimuli>(
		"/attention/remove_instances");

}

void
AttentionClient::update()
{
	std::vector<bica_graph::StringEdge> candidates = graph_.get_string_edges_by_data("want_see");
	std::set<std::string> instances_aux = instances_sent_;

	static bool inited = false;

	for (auto edge : candidates)
	{
		if (graph_.get_node(edge.get_source()).get_type() == "robot" &&
				graph_.get_node(edge.get_target()).get_type() == class_ &&
				graph_.exist_tf_edge(edge.get_source(), edge.get_target()))
		{
			gb_attention_msgs::AttentionPoints msg;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = edge.get_source();
			msg.class_id = class_;
			msg.instance_id = edge.get_target();

			std::list<geometry_msgs::Point> points = get_attention_points(edge);
			for (auto point : points)
					msg.attention_points.push_back(point);

			attention_points_pub_.publish(msg);
			instances_sent_.insert(edge.get_target());

			auto it = instances_aux.find(edge.get_target());
			if (it != instances_aux.end())
				instances_aux.erase(it);
		}
	}

	for (auto instance : instances_aux)
	{
		gb_attention_msgs::RemoveAttentionStimuli srv;
		srv.request.class_id = class_;
		srv.request.instance_id = instance;

		if (!remove_instance_service_.call(srv))
			ROS_ERROR("Error calling to remove stimuli service");

		auto it = instances_sent_.find(instance);
		if (it != instances_sent_.end())
			instances_sent_.erase(it);
	}
}

};  // namespace gb_attention
