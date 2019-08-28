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

#include "gb_attention/RoundRobinAttentionServer.h"

#include <string>
#include <list>


namespace gb_attention
{

RoundRobinAttentionServer::RoundRobinAttentionServer()
{
}

void
RoundRobinAttentionServer::update_points()
{
	attention_points_.sort(AttentionPointCompareRoundRobin());
}


void
RoundRobinAttentionServer::update()
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

	if ((ros::Time::now() - time_in_pos_).toSec() > (TIME_HEAD_TRAVEL + TIME_IN_POINT))
  {
		attention_points_.begin()->epoch++;

		update_points();
		publish_markers();

		goal_yaw_ = attention_points_.begin()->yaw;
		goal_pitch_ = attention_points_.begin()->pitch;

		joint_cmd_.points[0].positions[0] = goal_yaw_;
		joint_cmd_.points[0].positions[1] = goal_pitch_;

		joint_cmd_.header.stamp = ros::Time::now();
		joint_cmd_pub_.publish(joint_cmd_);
	}

	if (fabs(current_yaw_ - goal_yaw_) > FOVEA_YAW ||
			fabs(current_pitch_ - goal_pitch_) > FOVEA_PITCH)
		time_in_pos_ = ros::Time::now();
}


};  // namespace gb_attention
