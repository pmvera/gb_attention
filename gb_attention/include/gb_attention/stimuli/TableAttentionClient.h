#ifndef GB_ATTENTION_STIMULI_TABLEATTENTIONCLIENT_H
#define GB_ATTENTION_STIMULI_TABLEATTENTIONCLIENT_H

#include <ros/ros.h>
#include <gb_attention/AttentionClient.h>

namespace gb_attention
{

class TableAttentionClient: public AttentionClient
{
public:
	TableAttentionClient();

	std::list<geometry_msgs::Point> get_attention_points(const bica_graph::StringEdge& edge);
};

};  // namespace gb_attention

#endif  // GB_ATTENTION_STIMULI_TABLEATTENTIONCLIENT_H
