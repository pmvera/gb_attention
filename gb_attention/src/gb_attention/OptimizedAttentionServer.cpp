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

#include "gb_attention/OptimizedAttentionServer.h"

#include <string>
#include <list>


namespace gb_attention
{

OptimizedAttentionServer::OptimizedAttentionServer()
{
	ts_sent_ = ros::Time::now() - ros::Duration(5.0);
	time_in_pos_ = ros::Time::now();

	 nh_.param("time_in_point", time_in_point_, time_in_point_);


}

void
OptimizedAttentionServer::update_points()
{
	AttentionServer::update_points();

	attention_points_.sort(AttentionPointCompareOptimized(current_yaw_, current_pitch_));
}


void
OptimizedAttentionServer::update()
{
	if (attention_points_.empty())
  {
		if (head_mgr_disabled_)
		{
			enable_head_manager();
			head_mgr_disabled_ = false;
		}
		ROS_WARN("Empty attention_points");
		return;
	}
	else
	{
		if (!head_mgr_disabled_)
		{
			disable_head_manager();
			head_mgr_disabled_ = true;
		}
	}

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


	if (
		(ros::Time::now() - ts_sent_).toSec() > 5.0 ||
		(ros::Time::now() - time_in_pos_).toSec() > (time_head_travel_ + time_in_point_))
  {
		if ((ros::Time::now() - ts_sent_).toSec() > 10)
			ROS_WARN("Timeout in attention point. Skipping");

		attention_points_.begin()->epoch++;
		print();
		update_points();
		publish_markers();

		goal_yaw_ = attention_points_.begin()->yaw;
		goal_pitch_ = attention_points_.begin()->pitch;

		time_head_travel_ = fabs(current_yaw_ - goal_yaw_) + fabs(current_pitch_ - goal_pitch_);
		time_head_travel_ = std::max(1.0f, time_head_travel_); // Minumun, 1 second

		joint_cmd_.points[0].positions[0] = goal_yaw_;
		joint_cmd_.points[0].positions[1] = goal_pitch_;

		joint_cmd_.header.stamp = ros::Time::now();
		ts_sent_ = ros::Time::now();
		joint_cmd_pub_.publish(joint_cmd_);
	}

	if (
			fabs(current_yaw_ - goal_yaw_) > FOVEA_YAW ||
			fabs(current_pitch_ - goal_pitch_) > FOVEA_PITCH)
		time_in_pos_ = ros::Time::now();

}


};  // namespace gb_attention
