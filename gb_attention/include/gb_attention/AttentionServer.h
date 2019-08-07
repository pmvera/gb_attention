#ifndef GB_ATTENTION_ATTENTIONSERVER_H
#define GB_ATTENTION_ATTENTIONSERVER_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>

#include <bica_graph/graph_client.h>

#include <gb_attention_msgs/AttentionPoints.h>
#include <gb_attention_msgs/RemoveAttentionStimuli.h>

namespace gb_attention
{

typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef std::shared_ptr<PointHeadClient> PointHeadClientPtr;

class AttentionServer
{
public:
	AttentionServer();

	void update();

protected:
	ros::NodeHandle nh_;
	bica_graph::GraphClient graph_;

	PointHeadClientPtr pointHeadClient_;

	ros::Subscriber attention_points_sub_;
	ros::ServiceServer remove_instance_service_;

	std::map<std::string, gb_attention_msgs::AttentionPoints> attention_points_;

	void attention_point_callback(const gb_attention_msgs::AttentionPoints::ConstPtr& msg);
	bool remove_stimuli_callback(gb_attention_msgs::RemoveAttentionStimuli::Request  &req,
	         gb_attention_msgs::RemoveAttentionStimuli::Response &res);

};

};  // namespace gb_attention

#endif  // GB_ATTENTION_ATTENTIONSERVER_H
