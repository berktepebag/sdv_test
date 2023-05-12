cmake_minimum_required(VERSION 3.0.2)
project(sdv_test)

find_package(catkin REQUIRED COMPONENTS 
roscpp
std_msgs
nav_msgs
geometry_msgs
ackermann_msgs
rosbag
message_generation
dynamic_reconfigure
visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sdv_test
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(sdv_test_global_planner_node src/mpc_global_plan.cpp src/MPC.cpp src/path_utilities.cpp)
# add_dependencies(sdv_test_global_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(sdv_test_global_planner_node
#   ${catkin_LIBRARIES}
#   ipopt
# )

add_executable(mpc src/mpc.cpp src/MPC.cpp src/path_utilities.cpp)
target_link_libraries(mpc 
${catkin_LIBRARIES}
ipopt 
)
