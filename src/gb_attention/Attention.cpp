#include <ros/ros.h>

#include "gb_attention/Attention.h"

Attention::Attention()
: nh_()
{
	init_look_action_client();
}

void
Attention::init_look_action_client()
{
	actionClient_.reset( new PointHeadClient("/head_controller/point_head_action") );

  int iterations = 0, max_iterations = 3;
  // Wait for head controller action server to come up
  while( !actionClient_->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the point_head_action server to come up");
    ++iterations;
  }
}

void
Attention::update_attention_stimuli()
{
	std::vector<StringEdge> interest_edges = graph_.get_string_edges_from_node_by_data("leia", "want_see");

	for (auto edge : interest_edges)
	{

	}
}

bool
Attention::get_point_to_look(geometry_msgs::PointStamped* point)
{

}

void
Attention::update()
{
	update_attention_stimuli();

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
}
