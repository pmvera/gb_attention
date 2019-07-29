#ifndef GB_ATTENTION_ATTENTION_H
#define GB_ATTENTION_ATTENTION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <bica_graph/graph_client.h>

namespace gb_attention
{

typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

class Attention
{
public:
	Attention();

private:
	ros::NodeHandle nh_;
	bica_graph::GraphClient graph_;

	PointHeadClientPtr pointHeadClient_;

	std::list<std::string> lookfor_type_stimuli_;
	std::list<std::string> wantsee_stimuli_;

	geometry_msgs::PoseStamped current_point_;

	void update();
};

};  // namespace gb_attention

#endif  // GB_ATTENTION_ATTENTION_H
