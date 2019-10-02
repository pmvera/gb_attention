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

#ifndef GB_ATTENTION_ATTENTIONOCTOMAPCLIENT_H
#define GB_ATTENTION_ATTENTIONOCTOMAPCLIENT_H

#include <ros/ros.h>

#include <bica_graph/graph_client.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeNode.h>

#include <string>
#include <list>
#include <set>

namespace gb_attention
{

class AttentionOctomapClient
{
public:
	explicit AttentionOctomapClient(const std::string& class_id);

	void update();

	virtual std::list<geometry_msgs::PointStamped> get_attention_points(const bica_graph::StringEdge& edge);

protected:
	void publish_markers(const std::list<geometry_msgs::PointStamped>& points);

	ros::NodeHandle nh_;
	bica_graph::GraphClient graph_;

  std::string working_frame_;
  ros::Publisher attention_points_pub_;
	ros::Publisher markers_pub_;
	ros::ServiceClient remove_instance_service_;
	std::string class_;

	std::set<std::string> instances_sent_;

	std::map<std::string, std::list<geometry_msgs::PointStamped>> attention_points_;

	int marker_counter_id_;
};

};  // namespace gb_attention

#endif  // GB_ATTENTION_ATTENTIONOCTOMAPCLIENT_H
