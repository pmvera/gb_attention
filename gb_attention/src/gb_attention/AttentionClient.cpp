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

#include <gb_attention_msgs/AttentionPoints.h>
#include <gb_attention_msgs/RemoveAttentionStimuli.h>
#include <visualization_msgs/MarkerArray.h>

#include <bica_graph/exceptions.h>
#include "gb_attention/AttentionClient.h"

#include <string>
#include <list>
#include <vector>
#include <set>

namespace gb_attention
{

AttentionClient::AttentionClient(const std::string& class_id)
: nh_(),
	class_(class_id),
	marker_counter_id_(0)
{
	attention_points_pub_ = nh_.advertise<gb_attention_msgs::AttentionPoints>(
		"/attention/attention_points", 100);

	remove_instance_service_ = nh_.serviceClient<gb_attention_msgs::RemoveAttentionStimuli>(
		"/attention/remove_instances");

	markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/attention_markers", 100);

	init_attention_points();
}

std::vector<std::string> AttentionClient::tokenize(const std::string& text)
{
  std::vector<std::string> ret;
  size_t start = 0, end = 0;

  while (end != std::string::npos)
  {
    end = text.find(",", start);
    ret.push_back(text.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }
  return ret;
}

void
AttentionClient::init_attention_points()
{
	std::vector<std::string> classes;
	nh_.param("/attention_classes", classes, std::vector<std::string>());

	if (std::find(classes.begin(), classes.end(), class_) == classes.end())
  {
		ROS_ERROR("There is not point defined for class %s", class_.c_str());
		return;
  }

	std::vector<std::string> instances;
	nh_.param("/" + class_ + "_instances", instances, std::vector<std::string>());

	for (const auto instance : instances)
  {
		std::list<geometry_msgs::PointStamped> instance_coords;

		std::vector<std::string> coords;
		nh_.param("/" + instance, coords, std::vector<std::string>());

		for (const auto coord : coords)
    {
			geometry_msgs::PointStamped p;
			std::vector<std::string> coord_str = tokenize(coord);
			p.point.x = std::stod(coord_str[0]);
			p.point.y = std::stod(coord_str[1]);
			p.point.z = std::stod(coord_str[2]);

			instance_coords.push_back(p);
		}

		attention_points_[instance] = instance_coords;
	}
}

void
AttentionClient::publish_markers(const std::list<geometry_msgs::PointStamped>& points)
{
	if (markers_pub_.getNumSubscribers() == 0)
		return;

	visualization_msgs::MarkerArray msg;

	for (auto point : points)
  {
		visualization_msgs::Marker att_marker;

		att_marker.header.frame_id = point.header.frame_id;
		att_marker.header.stamp = point.header.stamp;
		att_marker.ns = ros::this_node::getName();
		att_marker.id = marker_counter_id_++;
		att_marker.type = visualization_msgs::Marker::SPHERE;
		att_marker.action = visualization_msgs::Marker::ADD;
		att_marker.pose.position.x = point.point.x;
		att_marker.pose.position.y = point.point.y;
		att_marker.pose.position.z = point.point.z;
		att_marker.pose.orientation.x = 0.0;
		att_marker.pose.orientation.y = 0.0;
		att_marker.pose.orientation.z = 0.0;
		att_marker.pose.orientation.w = 1.0;
		att_marker.scale.x = 0.1;
		att_marker.scale.y = 0.1;
		att_marker.scale.z = 0.1;
		att_marker.color.b = 0;
		att_marker.color.g = 255.0;
		att_marker.color.r = 0.0;
		att_marker.color.a = 0.6;
		att_marker.lifetime = ros::Duration(1.0);

		msg.markers.push_back(att_marker);
	}

	markers_pub_.publish(msg);
}

void
AttentionClient::update()
{
	std::vector<bica_graph::StringEdge> candidates = graph_.get_string_edges_by_data("want_see");
	std::set<std::string> instances_aux = instances_sent_;

	static bool inited = false;

	for (auto edge : candidates)
  {
		bool exist_tf = true;
		try
    {
			graph_.get_tf(edge.get_source(), edge.get_target());
		}
		catch(const bica_graph::exceptions::TransformNotPossible& e)
    {
			exist_tf = false;
		}

		if (graph_.get_node(edge.get_source()).get_type() == "robot" &&
				graph_.get_node(edge.get_target()).get_type() == class_ &&
				exist_tf)
	  {
			gb_attention_msgs::AttentionPoints msg;
			msg.class_id = class_;
			msg.instance_id = edge.get_target();

			std::list<geometry_msgs::PointStamped> points = get_attention_points(edge);

			publish_markers(points);

			for (auto point : points)
					msg.attention_points.push_back(point);

			attention_points_pub_.publish(msg);
			instances_sent_.insert(edge.get_target());

			auto it = instances_aux.find(edge.get_target());
			if (it != instances_aux.end())
				instances_aux.erase(it);
		}
	}

	for (auto instance : instances_aux)
  {
		gb_attention_msgs::RemoveAttentionStimuli srv;
		srv.request.class_id = class_;
		srv.request.instance_id = instance;

		if (!remove_instance_service_.call(srv))
			ROS_ERROR("Error calling to remove stimuli service");

		auto it = instances_sent_.find(instance);
		if (it != instances_sent_.end())
			instances_sent_.erase(it);
	}
}

std::list<geometry_msgs::PointStamped>
AttentionClient::get_attention_points(const bica_graph::StringEdge& edge)
{
	tf2::Transform trans;
	try
  {
		trans = graph_.get_tf(edge.get_source(), edge.get_target());
	}
	catch(const bica_graph::exceptions::TransformNotPossible& e)
  {
		return std::list<geometry_msgs::PointStamped>();
	}

	std::list<geometry_msgs::PointStamped> ret;

	if (attention_points_.find (edge.get_target()) != attention_points_.end())
  {
		ret = attention_points_[edge.get_target()];
	}
	else
  {
		ret = attention_points_[class_ + "_default"];
	}

	for (auto& point : ret)
  {
		point.header.stamp = ros::Time::now();
		point.header.frame_id = edge.get_target();
	}

	return ret;
}

};  // namespace gb_attention
