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

#include "gb_attention/AttentionServer.h"

#include <string>
#include <list>

#define TIME_IN_POINT	2.0

namespace gb_attention
{

AttentionServer::AttentionServer()
: nh_(),
	tf_listener_(tfBuffer_),
	current_yaw_(0.0),
	current_pitch_(0.0)
{
	joint_cmd_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 100);

	attention_points_sub_ = nh_.subscribe("/attention/attention_points", 100,
		&AttentionServer::attention_point_callback, this);

	remove_instance_service_ = nh_.advertiseService("/attention/remove_instances",
		&AttentionServer::remove_stimuli_callback, this);

	last_attention_point_sent_ = ros::Time::now();

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

bool
AttentionServer::remove_stimuli_callback(gb_attention_msgs::RemoveAttentionStimuli::Request &req,
				 gb_attention_msgs::RemoveAttentionStimuli::Response& res)
{
	remove_points(req.class_id, req.instance_id);

	return true;
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
  joint_state_.header.stamp = ros::Time::now();
  joint_state_.joint_names.resize(2);
  joint_state_.points.resize(1);

  joint_state_.joint_names[0] ="head_1_joint";
  joint_state_.joint_names[1] ="head_2_joint";

  joint_state_.points[0].positions.resize(2);
  joint_state_.points[0].velocities.resize(2);
  joint_state_.points[0].accelerations.resize(2);
  joint_state_.points[0].time_from_start = ros::Duration(1.0);

  joint_state_.points[0].positions[0] = 0.0;
  joint_state_.points[0].positions[1] = 0.0;
  joint_state_.points[0].velocities[0] = 0.5;
  joint_state_.points[0].velocities[1] = 0.0;
  joint_state_.points[0].accelerations[0] = 0.5;
  joint_state_.points[0].accelerations[1] = 0.0;
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
	}

	attention_points_.sort(AttentionPointCompare(current_yaw_, current_pitch_));
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
		attention_points_.begin()->epoch++;

		update_points();

		print();

		current_yaw_ = attention_points_.begin()->yaw;
		current_pitch_ = attention_points_.begin()->pitch;

		joint_state_.points[0].positions[0] = current_yaw_;
		joint_state_.points[0].positions[1] = current_pitch_;

		ROS_INFO("Commanding pan %lf", current_yaw_);
		ROS_INFO("Commanding tilt %lf", current_pitch_);

		joint_state_.header.stamp = ros::Time::now();
		joint_cmd_pub_.publish(joint_state_);
  }
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
