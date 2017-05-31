cmake_minimum_required(VERSION 2.8.3)
project(flight_code)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  rospy
  std_msgs
)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(takeoff_hold_test src/takeoff_hold_test.cpp)
#target_link_libraries(takeoff_hold_test ${catkin_LIBRARIES})

#add_executable(LeafBlowerTest src/LeafBlowerTest.cpp)
#target_link_libraries(LeafBlowerTest ${catkin_LIBRARIES})

#add_executable(roll_test src/x_roll_test.cpp)
#target_link_libraries(roll_test ${catkin_LIBRARIES})

#add_executable(vel_sys_id src/sys_id_vel.cpp)
#target_link_libraries(vel_sys_id ${catkin_LIBRARIES})

#add_executable(RC_override src/RC_override_test.cpp)
#target_link_libraries(RC_override ${catkin_LIBRARIES})

#add_executable(RC_sysID src/sys_ID_test.cpp)
#target_link_libraries(RC_sysID ${catkin_LIBRARIES})

add_executable(forward_vel_test src/forward_vel_test.cpp)
target_link_libraries(forward_vel_test ${catkin_LIBRARIES})
