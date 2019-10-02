cmake_minimum_required(VERSION 2.8.3)
project(gb_attention_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  octomap_msgs
)

add_message_files(
 FILES
 OctomapPoints.msg
 AttentionPoints.msg
)

add_service_files(
 FILES
 RemoveAttentionStimuli.srv
)


generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
 octomap_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  octomap_msgs
)
