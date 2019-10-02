cmake_minimum_required(VERSION 2.8.3)
project(gb_attention)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gb_attention_msgs
  geometry_msgs
  octomap_ros
  control_msgs
  bica_graph
  actionlib
  roslint
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/gb_attention/AttentionClient.cpp
  src/gb_attention/AttentionServer.cpp
  src/gb_attention/AttentionOctomapClient.cpp
  src/gb_attention/AttentionOctomapServer.cpp
  src/gb_attention/OptimizedAttentionServer.cpp
  src/gb_attention/OptimizedOctomapAttentionServer.cpp
  src/gb_attention/RoundRobinAttentionServer.cpp
  src/gb_attention/RoundRobinOctomapAttentionServer.cpp
  src/gb_attention/SimpleAttentionServer.cpp
  src/gb_attention/SimpleOctomapAttentionServer.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(attention_server_node src/attention_server_node.cpp)
target_link_libraries(attention_server_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(attention_server_octomap_node src/attention_server_octomap_node.cpp)
target_link_libraries(attention_server_octomap_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(attention_client_node src/attention_client_node.cpp)
target_link_libraries(attention_client_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(attention_octomap_client_node src/attention_octomap_client_node.cpp)
target_link_libraries(attention_octomap_client_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_attention_node tests/test_attention_node.cpp)
target_link_libraries(test_attention_node ${catkin_LIBRARIES} ${PROJECT_NAME})


add_executable(head_test_simple_node src/head_test_simple_node.cpp)
target_link_libraries(head_test_simple_node ${catkin_LIBRARIES} ${PROJECT_NAME})


install(TARGETS attention_server_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS attention_server_octomap_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

set(ROSLINT_CPP_OPTS "--filter=-whitespace/indent,-whitespace/tab,-runtime/references")

roslint_cpp(
  src/gb_attention/AttentionClient.cpp include/gb_attention/AttentionClient.h
  src/gb_attention/AttentionOctomapClient.cpp include/gb_attention/AttentionOctomapClient.h
  src/gb_attention/AttentionServer.cpp include/gb_attention/AttentionServer.h
  src/attention_client_node.cpp
  src/head_test_simple_node.cpp
  src/attention_server_node.cpp
  src/attention_server_octomap_node.cpp
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_attention tests/test_attention.cpp)
  target_link_libraries(test_attention ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()
