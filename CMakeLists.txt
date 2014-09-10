cmake_minimum_required(VERSION 2.8.3)
project(dumbo_cart_vel_controller)

find_package(catkin REQUIRED COMPONENTS roscpp brics_actuator control_msgs kdl_wrapper geometry_msgs kdl_conversions)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp brics_actuator control_msgs kdl_wrapper geometry_msgs kdl_conversions
  INCLUDE_DIRS include
  LIBRARIES dumbo_cart_vel_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(dumbo_cart_vel_controller_node src/dumbo_cart_vel_controller_node.cpp src/DumboCartVelController.cpp)
target_link_libraries(dumbo_cart_vel_controller_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(dumbo_cart_vel_controller src/DumboCartVelController.cpp)
target_link_libraries(dumbo_cart_vel_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS dumbo_cart_vel_controller_node dumbo_cart_vel_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)