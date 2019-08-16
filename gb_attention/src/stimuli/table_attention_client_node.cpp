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

#include "gb_attention/AttentionClient.h"
#include <list>

class TableAttentionClient: public gb_attention::AttentionClient
{
public:
	TableAttentionClient()
  : AttentionClient("table")
  {
  }

	std::list<geometry_msgs::PointStamped> get_attention_points(const bica_graph::StringEdge& edge)
  {
    std::list<geometry_msgs::PointStamped> ret;
    const tf2::Transform trans = graph_.get_tf_edge(edge.get_source(), edge.get_target()).get();

    geometry_msgs::PointStamped p1;

		p1.header.stamp = ros::Time::now();
		p1.header.frame_id = edge.get_source();
    p1.point.x = trans.getOrigin().x();
    p1.point.y = trans.getOrigin().y();
    p1.point.z = trans.getOrigin().z();

    ret.push_back(p1);

    return ret;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_attention_client_node");
  ros::NodeHandle n;

  TableAttentionClient table_attention_client;

  ros::Rate rate(1);

  while (ros::ok())
  {
    table_attention_client.update();

    ros::spinOnce();
    rate.sleep();
  }

   return 0;
 }
