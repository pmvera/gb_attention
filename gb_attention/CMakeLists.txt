cmake_minimum_required(VERSION 2.8.3)
project(gb_attention)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gb_attention_msgs
  geometry_msgs
  control_msgs
  bica_graph
  actionlib
  roslint
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
  src/gb_attention/stimuli/TableAttentionClient.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(attention_node
  src/attention_node.cpp
)

target_link_libraries(attention_node ${catkin_LIBRARIES} ${PROJECT_NAME})

install(TARGETS attention_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

set(ROSLINT_CPP_OPTS "--filter=-whitespace/indent,-whitespace/tab")

roslint_cpp(
  src/gb_attention/AttentionClient.cpp include/gb_attention/AttentionClient.h
  src/gb_attention/AttentionServer.cpp include/gb_attention/AttentionServer.h
  src/gb_attention/stimuli/TableAttentionClient.cpp include/gb_attention/stimuli/TableAttentionClient.h
  src/attention_node.cpp
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_attention tests/test_attention.cpp)
  target_link_libraries(test_attention ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()
