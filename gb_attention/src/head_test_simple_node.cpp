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
#include <tf2_ros/transform_listener.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <string>

void init_join_state(trajectory_msgs::JointTrajectory& joint_state)
{
  joint_state.header.stamp = ros::Time::now();
  joint_state.joint_names.resize(2);
  joint_state.points.resize(1);

  joint_state.joint_names[0] ="head_1_joint";
  joint_state.joint_names[1] ="head_2_joint";

  joint_state.points[0].positions.resize(2);
  joint_state.points[0].velocities.resize(2);
  joint_state.points[0].accelerations.resize(2);
  joint_state.points[0].time_from_start = ros::Duration(1.0);

  joint_state.points[0].positions[0] = 0.0;
  joint_state.points[0].positions[1] = 0.0;
  joint_state.points[0].velocities[0] = 0.5;
  joint_state.points[0].velocities[1] = 0.0;
  joint_state.points[0].accelerations[0] = 0.5;
  joint_state.points[0].accelerations[1] = 0.0;
}

tf2::Transform init_joint_tf(const std::string& link, const std::string& point_frame, tf2_ros::Buffer& tfBuffer)
{
  geometry_msgs::TransformStamped p2torso_msg;
  tf2::Transform point2torso;
  tf2::Transform torso2head1;
  tf2::Transform head12head;

  std::string error;
  if (tfBuffer.canTransform(point_frame, "torso_lift_link", ros::Time(0), ros::Duration(0.2), &error))
    p2torso_msg = tfBuffer.lookupTransform(point_frame, "torso_lift_link", ros::Time(0));
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

  if (link == "head_2_link")
    return point2torso * torso2head1 * head12head;
  else
    return point2torso * torso2head1;
}

double point_to_yaw(const tf2::Stamped<tf2::Vector3>& point, const tf2::Transform& tfh)
{
  tf2::Vector3 point_head = tfh.inverse() * point;

  ROS_INFO("Point (%lf, %lf, %lf) en head 1: (%lf, %lf, %lf) [%lf]", point.x(), point.y(), point.z(),
    point_head.x(), point_head.y(), point_head.z(), atan2(point_head.x(), point_head.y()));

  return atan2(point_head.y(), point_head.x());
}

double point_to_pitch(const tf2::Stamped<tf2::Vector3>& point, const tf2::Transform& tfh)
{
  tf2::Vector3 point_head = tfh.inverse() * point;

  ROS_INFO("Point (%lf, %lf, %lf) en head 2: (%lf, %lf, %lf) [%lf]", point.x(), point.y(), point.z(),
    point_head.x(), point_head.y(), point_head.z(), atan2(point_head.x(), point_head.x()));

  return atan2(point_head.z(), point_head.x());
}

void loop(float seconds, trajectory_msgs::JointTrajectory& joint_state, ros::Publisher& joint_cmd_pub)
{
  ros::Rate rate(1);
  ros::Time start = ros::Time::now();

  while (ros::ok() && (ros::Time::now() - start).toSec() < seconds)
  {
    joint_state.header.stamp = ros::Time::now();
    joint_cmd_pub.publish(joint_state);

    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "attention_node");
  ros::NodeHandle n;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf_listener(tfBuffer);

  ros::Publisher joint_cmd_pub = n.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 100);

  trajectory_msgs::JointTrajectory joint_state;
  init_join_state(joint_state);

  tf2::Stamped<tf2::Vector3> att_point;
  att_point.frame_id_ = "base_footprint";
  att_point.stamp_ = ros::Time::now();
  att_point.setX(2.0);
  att_point.setY(-2.0);
  att_point.setZ(0.0);

  tf2::Transform p2h1 = init_joint_tf("head_1_link", att_point.frame_id_, tfBuffer);
  tf2::Transform p2h2 = init_joint_tf("head_2_link", att_point.frame_id_, tfBuffer);

  double roll, pitch, yaw;
  tf2::Matrix3x3(p2h1.getRotation()).getRPY(roll, pitch, yaw);
  ROS_INFO("TF HEAD 1 (%lf, %lf, %lf) (r%lf, p%lf, y%lf)",
    p2h1.getOrigin().x(),
    p2h1.getOrigin().y(),
    p2h1.getOrigin().z(),
    roll, pitch, yaw);

  tf2::Matrix3x3(p2h2.getRotation()).getRPY(roll, pitch, yaw);
  ROS_INFO("TF HEAD 2 (%lf, %lf, %lf) (r%lf, p%lf, y%lf)",
    p2h2.getOrigin().x(),
    p2h2.getOrigin().y(),
    p2h2.getOrigin().z(),
    roll, pitch, yaw);

  yaw = point_to_yaw(att_point, p2h1);
  pitch = point_to_pitch(att_point, p2h2);

  joint_state.points[0].positions[0] = yaw;
  joint_state.points[0].positions[1] = pitch;
  ROS_INFO("Commanding pan %lf", yaw);
  ROS_INFO("Commanding tilt %lf", pitch);

  loop(3.0, joint_state, joint_cmd_pub);


  return 0;
 }
