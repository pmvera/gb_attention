/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2019, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include "gb_attention/AttentionServer.h"

#include <string>
#include <list>


namespace gb_attention
{

AttentionServer::AttentionServer()
: nh_(),
	tf_listener_(tfBuffer_),
	current_yaw_(0.0),
	current_pitch_(0.0),
	head_disable_ac_("/pal_head_manager/disable", true),
	head_mgr_disabled_(false),
	time_in_point_(1.0),
	time_head_travel_(2.0)
{
	joint_cmd_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 100);

	attention_points_sub_ = nh_.subscribe("/attention/attention_points", 100,
		&AttentionServer::attention_point_callback, this);

	joint_state_sub_ = nh_.subscribe("/joint_states", 1,
		&AttentionServer::joint_state_callback, this);

	remove_instance_service_ = nh_.advertiseService("/attention/remove_instances",
		&AttentionServer::remove_stimuli_callback, this);

	markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/attention_markers", 100);

	time_in_pos_ = ros::Time::now();

	init_join_state();
}

void
AttentionServer::attention_point_callback(const gb_attention_msgs::AttentionPoints::ConstPtr& msg)
{
	int point_counter = 0;
	for (auto msg_point : msg->attention_points)
  {
		std::string point_id = msg->class_id + "." + msg->instance_id + "." + std::to_string(point_counter++);

		bool found = false;
		std::list<AttentionPoint>::iterator it = attention_points_.begin();
		while (it != attention_points_.end() && !found)
	  {
			found = it->point_id == point_id;
			if (!found) it++;
		}

		if (found)
	  {
			tf2::fromMsg(msg_point, it->point);
		}
		else
	  {
			AttentionPoint att_point;
			att_point.point_id = point_id;

			if (attention_points_.empty())
				att_point.epoch = 0;
			else
				att_point.epoch = attention_points_.begin()->epoch;

			tf2::fromMsg(msg_point, att_point.point);

			attention_points_.push_back(att_point);
		}
	}
}

void
AttentionServer::update_points()
{
	for (auto& point : attention_points_)
  {
		geometry_msgs::TransformStamped p2torso_msg;
		tf2::Transform point2torso;
		tf2::Transform torso2head1;
		tf2::Transform head12head;

		std::string error;
		if (tfBuffer_.canTransform(point.point.frame_id_, "torso_lift_link",
			ros::Time(0), ros::Duration(0.1), &error))
			p2torso_msg = tfBuffer_.lookupTransform(point.point.frame_id_, "torso_lift_link", ros::Time(0));
		else
    {
			ROS_ERROR("Can't transform %s", error.c_str());
			continue;
		}
		tf2::Stamped<tf2::Transform> aux;
		tf2::convert(p2torso_msg, aux);

		point2torso = aux;

		torso2head1.setOrigin(tf2::Vector3(0.182, 0.0, 0.0));
		torso2head1.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
		head12head.setOrigin(tf2::Vector3(0.005, 0.0, 0.098));
		head12head.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

		tf2::Vector3 point_head_1 = (point2torso * torso2head1).inverse() * point.point;
		tf2::Vector3 point_head_2 = (point2torso * torso2head1 * head12head).inverse() * point.point;

		point.yaw = atan2(point_head_1.y(), point_head_1.x());
		point.pitch = atan2(point_head_1.z(), point_head_1.x());

		/*ROS_INFO("Point (%lf %lf %lf) (%lf %lf %lf)  =======> (%lf %lf)",
			point_head_1.x(), point_head_1.y(), point_head_1.z(),
			point_head_2.x(), point_head_2.y(), point_head_2.z(),
			point.yaw, point.pitch);*/
	}
}

bool
AttentionServer::remove_stimuli_callback(gb_attention_msgs::RemoveAttentionStimuli::Request &req,
				 gb_attention_msgs::RemoveAttentionStimuli::Response& res)
{
	remove_points(req.class_id, req.instance_id);

	return true;
}

void
AttentionServer::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (int i = 0; i < msg->name.size(); i++)
  {
		if (msg->name[i] == "head_1_joint")
			current_yaw_ = msg->position[i];
		if (msg->name[i] == "head_2_joint")
			current_pitch_ = msg->position[i];
	}
}


void
AttentionServer::remove_points(const std::string& class_id, const std::string& instance_id)
{
	std::string point_id = class_id + "." + instance_id;

	auto it = attention_points_.begin();
	while (it != attention_points_.end())
  {
		if (it->point_id.rfind(point_id, 0) == 0)  // If it->point_id starts with point_id
			it = attention_points_.erase(it);
		else
			++it;
	}
}

void
AttentionServer::init_join_state()
{
  joint_cmd_.header.stamp = ros::Time::now();
  joint_cmd_.joint_names.resize(2);
  joint_cmd_.points.resize(1);

  joint_cmd_.joint_names[0] ="head_1_joint";
  joint_cmd_.joint_names[1] ="head_2_joint";

  joint_cmd_.points[0].positions.resize(2);
  joint_cmd_.points[0].velocities.resize(2);
  joint_cmd_.points[0].accelerations.resize(2);
  joint_cmd_.points[0].time_from_start = ros::Duration(time_head_travel_);

  joint_cmd_.points[0].positions[0] = 0.0;
  joint_cmd_.points[0].positions[1] = 0.0;
  joint_cmd_.points[0].velocities[0] = NECK_SPEED;
  joint_cmd_.points[0].velocities[1] = NECK_SPEED;
  joint_cmd_.points[0].accelerations[0] = NECK_SPEED;
  joint_cmd_.points[0].accelerations[1] = NECK_SPEED;
}


void
AttentionServer::publish_markers()
{
	if (markers_pub_.getNumSubscribers() == 0)
		return;

	tf2::Stamped<tf2::Vector3> end_point = attention_points_.begin()->point;


	visualization_msgs::MarkerArray msg;
	visualization_msgs::Marker att_marker;

	att_marker.header.frame_id = "xtion_link";
	att_marker.header.stamp = ros::Time::now();
	att_marker.ns = ros::this_node::getName();
	att_marker.id = 0;
	att_marker.type = visualization_msgs::Marker::ARROW;
	att_marker.action = visualization_msgs::Marker::ADD;
	att_marker.scale.x = 0.01;
	att_marker.scale.y = 0.1;
	att_marker.scale.z = 0.1;
	att_marker.color.b = 0;
	att_marker.color.g = 0;
	att_marker.color.r = 255;
	att_marker.color.a = 1.0;
	att_marker.lifetime = ros::Duration(5.0);

	geometry_msgs::Point start, end;
	start.x = 0.0;
	start.y = 0.0;
	start.z = 0.0;

	geometry_msgs::TransformStamped tf_msg;
	std::string error;
	if (tfBuffer_.canTransform(end_point.frame_id_, "xtion_link",
		ros::Time(0), ros::Duration(0.1), &error))
		tf_msg = tfBuffer_.lookupTransform(end_point.frame_id_, "xtion_link", ros::Time(0));
	else
  {
		ROS_ERROR("Can't transform %s", error.c_str());
	}
	tf2::Transform tf;
	tf2::Stamped<tf2::Transform> aux;
	tf2::convert(tf_msg, aux);
	tf = aux;

	tf2::Vector3 point_end = tf.inverse() *  end_point;

	end.x = point_end.x();
	end.y = point_end.y();
	end.z = point_end.z();

	att_marker.points.push_back(start);
	att_marker.points.push_back(end);


	msg.markers.push_back(att_marker);

	markers_pub_.publish(msg);
}

void
AttentionServer::enable_head_manager()
{
	ROS_INFO("Enabling head manager");
	head_disable_ac_.cancelGoal();
}

void
AttentionServer::disable_head_manager()
{
	ROS_INFO("Disabling head manager");

	pal_common_msgs::DisableGoal goal;
 	goal.duration = 0.0;
 	head_disable_ac_.sendGoal(goal);
}

void
AttentionServer::print()
{
	ROS_INFO("===============================");
	for (auto points : attention_points_)
  {
		ROS_INFO("[%d] [%s]", points.epoch, points.point_id.c_str());
		ROS_INFO("\t[%s] (%lf, %lf, %lf) [%lf, %lf]", points.point.frame_id_.c_str(),
			points.point.x(), points.point.y(), points.point.z(), points.yaw, points.pitch);
	}
}

};  // namespace gb_attention
