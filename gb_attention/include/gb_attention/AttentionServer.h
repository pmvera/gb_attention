#ifndef GB_ATTENTION_ATTENTIONSERVER_H
#define GB_ATTENTION_ATTENTIONSERVER_H

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <bica_graph/graph_client.h>

#include <gb_attention_msgs/AttentionPoints.h>
#include <gb_attention_msgs/RemoveAttentionStimuli.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace gb_attention
{

struct AttentionPoint
{
	std::string class_id;
	std::string instance_id;
	tf2::Stamped<tf2::Vector3> point;
};

class AttentionServer
{
public:
	AttentionServer();

	void update();

protected:
	ros::NodeHandle nh_;
	bica_graph::GraphClient graph_;

	tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tf_listener_;

	ros::Publisher joint_cmd_pub_;

	ros::Subscriber attention_points_sub_;
	ros::ServiceServer remove_instance_service_;

	std::map<std::string, AttentionPoint> attention_points_;
	std::map<std::string, AttentionPoint>::iterator it_attention_points_;
	ros::Time last_attention_point_sent_;

	void attention_point_callback(const gb_attention_msgs::AttentionPoints::ConstPtr& msg);
	bool remove_stimuli_callback(gb_attention_msgs::RemoveAttentionStimuli::Request  &req,
	         gb_attention_msgs::RemoveAttentionStimuli::Response &res);

	void init_join_state(trajectory_msgs::JointTrajectory& joint_state);
	tf2::Transform init_joint_tf(const std::string& link, const std::string& point_frame, tf2_ros::Buffer& tfBuffer);
	double point_to_yaw(const tf2::Stamped<tf2::Vector3>& point, const tf2::Transform& tfh);
	double point_to_pitch(const tf2::Stamped<tf2::Vector3>& point, const tf2::Transform& tfh);

};

};  // namespace gb_attention

#endif  // GB_ATTENTION_ATTENTIONSERVER_H
