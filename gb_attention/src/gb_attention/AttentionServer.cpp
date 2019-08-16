#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "gb_attention/AttentionServer.h"


#define TIME_IN_POINT	2.0

namespace gb_attention
{

AttentionServer::AttentionServer()
: nh_(),
	tf_listener_(tfBuffer_)
{
	joint_cmd_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 100);

	attention_points_sub_ = nh_.subscribe("/attention/attention_points", 100,
		&AttentionServer::attention_point_callback, this);

	remove_instance_service_ = nh_.advertiseService("/attention/remove_instances",
		&AttentionServer::remove_stimuli_callback, this);

	it_attention_points_ = attention_points_.begin();
	last_attention_point_sent_ = ros::Time::now();
}

void
AttentionServer::attention_point_callback(const gb_attention_msgs::AttentionPoints::ConstPtr& msg)
{
	for (auto msg_point : msg->attention_points)
	{
		AttentionPoint att_point;
		att_point.class_id = msg->class_id;
		att_point.instance_id = msg->instance_id;

		tf2::fromMsg(msg_point, att_point.point);

		attention_points_[msg->instance_id] = att_point;
	}
}

bool
AttentionServer::remove_stimuli_callback(gb_attention_msgs::RemoveAttentionStimuli::Request  &req,
				 gb_attention_msgs::RemoveAttentionStimuli::Response &res)
{
	if (req.instance_id == "")
	{
		auto it = attention_points_.begin();
		while (it != attention_points_.end())
		{
			if (it->second.class_id == req.class_id)
				it = attention_points_.erase(it);
			else
				++it;
		}
	}
	else
		attention_points_.erase(req.instance_id);

	return true;
}

void
AttentionServer::init_join_state(trajectory_msgs::JointTrajectory& joint_state)
{
  joint_state.header.stamp = ros::Time::now();
  joint_state.joint_names.resize(2);
  joint_state.points.resize(1);

  joint_state.joint_names[0] ="head_1_joint";
  joint_state.joint_names[1] ="head_2_joint";

  joint_state.points[0].positions.resize(2);
  joint_state.points[0].velocities.resize(2);
  joint_state.points[0].accelerations.resize(2);
  joint_state.points[0].time_from_start = ros::Duration(1.0);

  joint_state.points[0].positions[0] = 0.0;
  joint_state.points[0].positions[1] = 0.0;
  joint_state.points[0].velocities[0] = 0.5;
  joint_state.points[0].velocities[1] = 0.0;
  joint_state.points[0].accelerations[0] = 0.5;
  joint_state.points[0].accelerations[1] = 0.0;
}

tf2::Transform
AttentionServer::init_joint_tf(const std::string& link, const std::string& point_frame, tf2_ros::Buffer& tfBuffer)
{
  geometry_msgs::TransformStamped p2torso_msg;
  tf2::Transform point2torso;
  tf2::Transform torso2head1;
  tf2::Transform head12head;

  std::string error;
  if (tfBuffer.canTransform(point_frame, "torso_lift_link", ros::Time(0), ros::Duration(0.2), &error))
    p2torso_msg = tfBuffer.lookupTransform(point_frame, "torso_lift_link", ros::Time(0));
  else
  {
    ROS_ERROR("Can't transform %s", error.c_str());
  }
  tf2::Stamped<tf2::Transform> aux;
  tf2::convert(p2torso_msg, aux);

  point2torso = aux;

  torso2head1.setOrigin(tf2::Vector3(0.182, 0.0, 0.0));
  torso2head1.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  head12head.setOrigin(tf2::Vector3(0.005, 0.0, 0.098));
  head12head.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  if (link == "head_2_link")
    return point2torso * torso2head1 * head12head;
  else
    return point2torso * torso2head1;
}

double
AttentionServer::point_to_yaw(const tf2::Stamped<tf2::Vector3>& point, const tf2::Transform& tfh)
{
  tf2::Vector3 point_head = tfh.inverse() * point;

  ROS_INFO("Point (%lf, %lf, %lf) en head 1: (%lf, %lf, %lf) [%lf]", point.x(), point.y(), point.z(),
    point_head.x(), point_head.y(), point_head.z(), atan2(point_head.x(), point_head.y()));

  return atan2(point_head.y(), point_head.x());
}

double
AttentionServer::point_to_pitch(const tf2::Stamped<tf2::Vector3>& point, const tf2::Transform& tfh)
{
  tf2::Vector3 point_head = tfh.inverse() * point;

  ROS_INFO("Point (%lf, %lf, %lf) en head 2: (%lf, %lf, %lf) [%lf]", point.x(), point.y(), point.z(),
    point_head.x(), point_head.y(), point_head.z(), atan2(point_head.x(), point_head.x()));

  return atan2(point_head.z(), point_head.x());
}

void
AttentionServer::update()
{
	ROS_INFO("=================================================");

	if (attention_points_.empty())
	{
		ROS_WARN("Empty attention_points");
		return;
	}

	if ((ros::Time::now() - last_attention_point_sent_).toSec() > TIME_IN_POINT)
	{
		it_attention_points_++;
		if (it_attention_points_ == attention_points_.end())
			it_attention_points_ = attention_points_.begin();

		last_attention_point_sent_ = ros::Time::now();
	
		tf2::Stamped<tf2::Vector3> att_point = it_attention_points_->second.point;

		trajectory_msgs::JointTrajectory joint_state;
		init_join_state (joint_state);

		tf2::Transform p2h1 = init_joint_tf("head_1_link", att_point.frame_id_, tfBuffer_);
		tf2::Transform p2h2 = init_joint_tf("head_2_link", att_point.frame_id_, tfBuffer_);

		double roll, pitch, yaw;
		tf2::Matrix3x3(p2h1.getRotation()).getRPY(roll, pitch, yaw);
		ROS_INFO("TF HEAD 1 (%lf, %lf, %lf) (r%lf, p%lf, y%lf)",
			p2h1.getOrigin().x(),
			p2h1.getOrigin().y(),
			p2h1.getOrigin().z(),
			roll, pitch, yaw);

		tf2::Matrix3x3(p2h2.getRotation()).getRPY(roll, pitch, yaw);
		ROS_INFO("TF HEAD 2 (%lf, %lf, %lf) (r%lf, p%lf, y%lf)",
			p2h2.getOrigin().x(),
			p2h2.getOrigin().y(),
			p2h2.getOrigin().z(),
			roll, pitch, yaw);

		yaw = point_to_yaw(att_point, p2h1);
		pitch = point_to_pitch(att_point, p2h2);

		joint_state.points[0].positions[0] = yaw;
		joint_state.points[0].positions[1] = pitch;
		ROS_INFO("Commanding pan %lf", yaw);
		ROS_INFO("Commanding tilt %lf", pitch);

		joint_state.header.stamp = ros::Time::now();
		joint_cmd_pub_.publish(joint_state);
	}

	/*
	ROS_INFO("===============================");
	for (auto points : attention_points_)
	{
		ROS_INFO("[%s / %s]", points.second.class_id.c_str(), points.second.instance_id.c_str());
		ROS_INFO("\t[%s] (%lf, %lf, %lf)", points.second.point.frame_id_.c_str(),
			points.second.point.x(), points.second.point.y(), points.second.point.z());
	}*/
}



};  // namespace gb_attention
