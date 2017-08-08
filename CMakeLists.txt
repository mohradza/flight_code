cmake_minimum_required(VERSION 2.8.3)
project(flight_code)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  rospy
  std_msgs
  message_runtime
  small_object
)

add_message_files(DIRECTORY msg 
                  FILES 
                  YawRateCmdMsg.msg
                  ControllerOutMsg.msg
                  AllDataOutMsg.msg 
                  YawAngleMsg.msg
                  FOFAllDataOutMsg.msg
                  DSwitchMsg.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

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

#add_executable(forward_vel_test src/main_flight_controller.cpp)
#target_link_libraries(forward_vel_test ${catkin_LIBRARIES})

#add_executable(takeoff_test src/forward_vel_stop_test.cpp)
#target_link_libraries(takeoff_test ${catkin_LIBRARIES})

#add_executable(pub_forward_vel src/pub_forward_vel.cpp)
#target_link_libraries(pub_forward_vel ${catkin_LIBRARIES})

add_executable(small_object_flight_code src/small_object_avoidance.cpp)
target_link_libraries(small_object_flight_code ${catkin_LIBRARIES})

add_executable(data_pub src/data_collect_and_pub.cpp)
target_link_libraries(data_pub ${catkin_LIBRARIES})

add_executable(FOF_data_pub src/FOF_data_collect_and_pub.cpp)
target_link_libraries(FOF_data_pub ${catkin_LIBRARIES})

