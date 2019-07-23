cmake_minimum_required(VERSION 2.8.3)
project(gb_attention)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(class_lib src/gb_attention/Class.cpp)
add_executable(nodo_sub_obj_class src/nodo_sub_obj.cpp)

target_link_libraries(nodo_sub_obj_class ${catkin_LIBRARIES} class_lib)

install(TARGETS nodo_sub_obj_class
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
  src/nodo_sub_obj.cpp
  src/gb_attention/Class.cpp
  include/gb_attention/Class.cpp
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_class tests/test_class.cpp)
  target_link_libraries(test_class ${catkin_LIBRARIES} class_lib)
endif()
