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
/* Author: Pablo Moreno Vera */

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include "gb_attention/SimpleOctomapAttentionServer.h"

#include <string>
#include <list>


namespace gb_attention
{

#define FACTOR_RADS	4.0

SimpleOctomapAttentionServer::SimpleOctomapAttentionServer()
{
	direction_yaw_ = 1.0f;
	direction_pitch_ = 1.0f;

	inited_ = false;
	ts_sent_ = ros::Time::now();
}

void
SimpleOctomapAttentionServer::update()
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

	if (!inited_ || (ros::Time::now() - ts_sent_) > duration_)
	{
		inited_ = true;
		ts_sent_ = ros::Time::now();

		update_points();
		update_limits();

		duration_ = ros::Duration(fabs(max_yaw_ - min_yaw_) * FACTOR_RADS + 1.0);

		trajectory_msgs::JointTrajectory joint_cmd;
		joint_cmd.header.stamp = ros::Time::now();
		joint_cmd.joint_names.resize(2);
		joint_cmd.points.resize(2);

		joint_cmd.joint_names[0] ="head_1_joint";
		joint_cmd.joint_names[1] ="head_2_joint";

		//  Vertical movement
		joint_cmd.points[0].positions.resize(2);
  	joint_cmd.points[0].velocities.resize(2);
  	joint_cmd.points[0].accelerations.resize(2);
  	joint_cmd.points[0].time_from_start = ros::Duration(1.0);

		if (current_pitch_ > max_pitch_) direction_pitch_ = -1.0;
		if (current_pitch_ < min_pitch_) direction_pitch_ = 1.0;

		joint_cmd.points[0].positions[0] = current_yaw_;
		joint_cmd.points[0].positions[1] = current_pitch_ + direction_pitch_ * FOVEA_PITCH;

		joint_cmd.points[0].velocities[0] = NECK_SPEED;
		joint_cmd.points[0].velocities[1] = 0.0;
		joint_cmd.points[0].accelerations[0] = NECK_SPEED;
		joint_cmd.points[0].accelerations[1] = 0.0;

		//  horizontal movement
		joint_cmd.points[1].positions.resize(2);
		joint_cmd.points[1].velocities.resize(2);
		joint_cmd.points[1].accelerations.resize(2);
		joint_cmd.points[1].time_from_start = duration_;

		if (direction_yaw_ < 0)
			joint_cmd.points[1].positions[0] = min_yaw_;
		else
			joint_cmd.points[1].positions[0] = max_yaw_;
		direction_yaw_ = - direction_yaw_;

		if (current_pitch_ > max_pitch_) direction_pitch_ = -1.0;
		if (current_pitch_ < min_pitch_) direction_pitch_ = 1.0;

		joint_cmd.points[1].positions[1] = 	joint_cmd.points[0].positions[1];

		joint_cmd.points[1].velocities[0] = NECK_SPEED;
		joint_cmd.points[1].velocities[1] = 0.0;
		joint_cmd.points[1].accelerations[0] = NECK_SPEED;
		joint_cmd.points[1].accelerations[1] = 0.0;

		joint_cmd_pub_.publish(joint_cmd);
	}

	publish_markers();

}

void
SimpleOctomapAttentionServer::update_limits()
{
	max_yaw_ = -std::numeric_limits<float>::max();
	max_pitch_ = -std::numeric_limits<float>::max();
	min_yaw_ = std::numeric_limits<float>::max();
	min_pitch_ = std::numeric_limits<float>::max();

	for (auto point : attention_points_)
	{
		max_yaw_ = std::max<float>(max_yaw_, point.yaw);
		max_pitch_ = std::max<float>(max_pitch_, point.pitch);
		min_yaw_ = std::min<float>(min_yaw_, point.yaw);
		min_pitch_ = std::min<float>(min_pitch_, point.pitch);
	}

}


};  // namespace gb_attention
