cmake_minimum_required(VERSION 2.8.3)
project(flight_code)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_runtime
  geometry_msgs
  mavros
  object_avoidance_cpp
)

add_message_files(DIRECTORY msg 
                  FILES 
                  YawRateCmdMsg.msg
                  ControllerOutMsg.msg
                  FlowRingOutMsg.msg
                  AllDataOutMsg.msg 
                  YawAngleMsg.msg
                  FOFAllDataOutMsg.msg
                  DSwitchMsg.msg)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS roscpp message_runtime std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})


#add_executable(small_object_flight_code_vicon src/small_object_avoidance_vicon.cpp)
#target_link_libraries(small_object_flight_code_vicon ${catkin_LIBRARIES})

#add_executable(FOF_data_pub_vicon src/FOF_vicon_data_collect_and_pub.cpp)
#add_dependencies(FOF_data_pub_vicon flight_code_generate_messages_cpp)
#target_link_libraries(FOF_data_pub_vicon ${catkin_LIBRARIES})

#add_executable(small_object_flight_code_vicon src/small_object_avoidance_vicon.cpp)
#add_dependencies(small_object_flight_code_vicon ${catkin_EXPORTED_TARGETS})
#target_link_libraries(small_object_flight_code_vicon ${catkin_LIBRARIES})

add_executable(small_object_flight_code_PX4Flow src/small_object_avoidance_PX4Flow.cpp)
add_dependencies(small_object_flight_code_PX4Flow ${catkin_EXPORTED_TARGETS})
target_link_libraries(small_object_flight_code_PX4Flow ${catkin_LIBRARIES})

#add_executable(FOF src/FOF_pub.cpp)
#add_dependencies(FOF ${catkin_EXPORTED_TARGETS})
#target_link_libraries(FOF ${catkin_libraries})

#add_executable(offb_test src/offboard_test.cpp)
#add_dependencies(offb_test flight_code_generate_messages_cpp)
#target_link_libraries(offb_test ${catkin_libraries})
