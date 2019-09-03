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

#ifndef GB_ATTENTION_ATTENTIONSERVER_H
#define GB_ATTENTION_ATTENTIONSERVER_H

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <bica_graph/graph_client.h>

#include <gb_attention_msgs/AttentionPoints.h>
#include <gb_attention_msgs/RemoveAttentionStimuli.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <pal_common_msgs/DisableGoal.h>
#include <pal_common_msgs/DisableAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <list>
#include <string>

namespace gb_attention
{


#define TIME_HEAD_TRAVEL	2.0
#define TIME_IN_POINT	1.0
#define NECK_SPEED	0.1
#define H_FOV (58.0 * M_PI / 180.0)
#define V_FOV (45.0 * M_PI / 180.0)
#define FOVEA_YAW (H_FOV / 2.0)
#define FOVEA_PITCH (V_FOV / 2.0)

struct AttentionPoint
{
	std::string point_id;
	tf2::Stamped<tf2::Vector3> point;
	float yaw;
	float pitch;
	int epoch;
};

class AttentionServer
{
public:
	AttentionServer();

	virtual void update() = 0;

protected:
	ros::NodeHandle nh_;
	bica_graph::GraphClient graph_;

	tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tf_listener_;

	ros::Publisher joint_cmd_pub_;
	ros::Publisher markers_pub_;

	ros::Subscriber attention_points_sub_;
	ros::Subscriber joint_state_sub_;
	ros::ServiceServer remove_instance_service_;

	actionlib::SimpleActionClient<pal_common_msgs::DisableAction> head_disable_ac_;

	std::list<AttentionPoint> attention_points_;

	ros::Time time_in_pos_;

	trajectory_msgs::JointTrajectory joint_cmd_;
	sensor_msgs::JointState joint_state_;

	virtual void update_points();

	void attention_point_callback(const gb_attention_msgs::AttentionPoints::ConstPtr& msg);
	void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);
	bool remove_stimuli_callback(gb_attention_msgs::RemoveAttentionStimuli::Request &req,
	         gb_attention_msgs::RemoveAttentionStimuli::Response& res);

	void remove_points(const std::string& class_id, const std::string& instance_id);

	void init_join_state();
	void print();
	void publish_markers();

	void enable_head_manager();
	void disable_head_manager();

	float current_yaw_;
	float current_pitch_;
	float goal_yaw_;
	float goal_pitch_;

	bool head_mgr_disabled_;
};

};  // namespace gb_attention

#endif  // GB_ATTENTION_ATTENTIONSERVER_H
