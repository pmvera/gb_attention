#include <ros/ros.h>

#include "gb_attention/AttentionServer.h"

namespace gb_attention
{

AttentionServer::AttentionServer()
: nh_()
{
	attention_points_sub_ = nh_.subscribe("/attention/attention_points", 100,
		&AttentionServer::attention_point_callback, this);

	remove_instance_service_ = nh_.advertiseService("/attention/remove_instances",
		&AttentionServer::remove_stimuli_callback, this);

}

void
AttentionServer::attention_point_callback(const gb_attention_msgs::AttentionPoints::ConstPtr& msg)
{
	attention_points_[msg->instance_id] = *msg;
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

/*
void
Attention::init_look_action_client()
{
	actionClient_ = std::make_shared<PointHeadClient>("/head_controller/point_head_action");

  int iterations = 0, max_iterations = 3;
  // Wait for head controller action server to come up
  while( !actionClient_->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the point_head_action server to come up");
    ++iterations;
  }
}*/


void
AttentionServer::update()
{
	ROS_INFO("===============================");
	for (auto points : attention_points_)
	{
		ROS_INFO("[%s / %s]", points.second.class_id.c_str(), points.second.instance_id.c_str());
		for (auto point : points.second.attention_points)
		{
			ROS_INFO("\t(%lf, %lf, %lf)", point.x, point.y, point.z);
		}
	}

/*	update_attention_stimuli();

	geometry_msgs::PointStamped current_point_;

	if (get_point_to_look(&current_point_))
	{
		control_msgs::PointHeadGoal goal;
  	//the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
  	goal.pointing_frame = current_point_.header.frame_id;
  	goal.pointing_axis.x = 0.0;
  	goal.pointing_axis.y = 0.0;
  	goal.pointing_axis.z = 1.0;
  	goal.min_duration = ros::Duration(1.0);
  	goal.max_velocity = 0.25;
  	goal.target = pointStamped;

  	pointHeadClient_->sendGoal(goal);
	}
	*/
}

};  // namespace gb_attention
