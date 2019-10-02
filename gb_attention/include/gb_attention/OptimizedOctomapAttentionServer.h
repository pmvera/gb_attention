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

#ifndef GB_ATTENTION_OPTIMIZEDOCTOMAPATTENTIONSERVER_H
#define GB_ATTENTION_OPTIMIZEDOCTOMAPATTENTIONSERVER_H

#include <ros/ros.h>

#include "gb_attention/AttentionOctomapServer.h"

#include <list>
#include <string>

namespace gb_attention
{

class AttentionPointCompareOptimized
{
public:
	AttentionPointCompareOptimized(float ref_yaw, float ref_pitch)
  : ref_yaw_(ref_yaw), ref_pitch_(ref_pitch) {}

	bool operator()(const AttentionOctomapPoint& a, const AttentionOctomapPoint& b)
  {
		if (a.epoch < b.epoch)
			return true;
		else if (b.epoch < a.epoch)
			return false;
		else
			return ((fabs(a.yaw - ref_yaw_) + fabs(a.pitch - ref_pitch_)) <
							(fabs(b.yaw - ref_yaw_) + fabs(b.pitch - ref_pitch_)));
	}

	float ref_yaw_;
	float ref_pitch_;
};


class OptimizedOctomapAttentionServer: public AttentionOctomapServer
{
public:
	OptimizedOctomapAttentionServer();

	void update();

protected:
	ros::Time ts_sent_;

	void update_points();
};

};  // namespace gb_attention

#endif  // GB_ATTENTION_OPTIMIZEDOCTOMAPATTENTIONSERVER_H
