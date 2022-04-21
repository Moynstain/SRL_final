cmake_minimum_required(VERSION 3.0.2)
project(srl_final)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    message_filters
)

add_message_files(
    FILES
    glove.msg
    Float32MultiArraySRL.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} )
include_directories(include /usr/include/eigen3/Eigen/src/Core)
include_directories(include /home/harlab/ros_ws_1/devel/include/srl_final)
# include_directories(/home/harlab/ros_ws_1/src/sense_glove_test/include/sense_glove_test)

# set(LIB_DIR_CONNECT /home/harlab/ros_ws_1/src/sense_glove_test/include/libSGCoreCpp.so)

add_executable(2_final_trans_all src/2_final_trans_all.cpp)
target_link_libraries(2_final_trans_all ${LIB_DIR_CONNECT})
target_link_libraries(2_final_trans_all ${catkin_LIBRARIES})
add_dependencies(2_final_trans_all srl_final_generate_messages_cpp)


add_executable(2_final_trans_all_backup src/2_final_trans_all_backup.cpp)
target_link_libraries(2_final_trans_all_backup ${LIB_DIR_CONNECT})
target_link_libraries(2_final_trans_all_backup ${catkin_LIBRARIES})
add_dependencies(2_final_trans_all_backup srl_final_generate_messages_cpp)

add_executable(group_discussion_sub src/group_discussion_sub.cpp)
target_link_libraries(group_discussion_sub ${LIB_DIR_CONNECT})
target_link_libraries(group_discussion_sub ${catkin_LIBRARIES})
add_dependencies(group_discussion_sub srl_final_generate_messages_cpp)

add_executable(group_discussion_pub src/group_discussion_pub.cpp)
target_link_libraries(group_discussion_pub ${LIB_DIR_CONNECT})
target_link_libraries(group_discussion_pub ${catkin_LIBRARIES})
add_dependencies(group_discussion_pub srl_final_generate_messages_cpp)

add_executable(group_discussion_eye_tracker_sim src/group_discussion_eye_tracker_sim.cpp)
target_link_libraries(group_discussion_eye_tracker_sim ${LIB_DIR_CONNECT})
target_link_libraries(group_discussion_eye_tracker_sim ${catkin_LIBRARIES})
add_dependencies(group_discussion_eye_tracker_sim srl_final_generate_messages_cpp)

add_executable(group_discussion_t265_no1_sim src/group_discussion_t265_no1_sim.cpp)
target_link_libraries(group_discussion_t265_no1_sim ${LIB_DIR_CONNECT})
target_link_libraries(group_discussion_t265_no1_sim ${catkin_LIBRARIES})
add_dependencies(group_discussion_t265_no1_sim srl_final_generate_messages_cpp)

add_executable(group_discussion_t265_no2_sim src/group_discussion_t265_no2_sim.cpp)
target_link_libraries(group_discussion_t265_no2_sim ${LIB_DIR_CONNECT})
target_link_libraries(group_discussion_t265_no2_sim ${catkin_LIBRARIES})
add_dependencies(group_discussion_t265_no2_sim srl_final_generate_messages_cpp)

add_executable(group_discussion_trans_all_sim src/group_discussion_trans_all_sim.cpp)
target_link_libraries(group_discussion_trans_all_sim ${LIB_DIR_CONNECT})
target_link_libraries(group_discussion_trans_all_sim ${catkin_LIBRARIES})
add_dependencies(group_discussion_trans_all_sim srl_final_generate_messages_cpp)

add_executable(group_discussion_srl_command src/group_discussion_srl_command.cpp)
target_link_libraries(group_discussion_srl_command ${LIB_DIR_CONNECT})
target_link_libraries(group_discussion_srl_command ${catkin_LIBRARIES})
add_dependencies(group_discussion_srl_command srl_final_generate_messages_cpp)
#add_executable(grammer_test src/grammer_test.cpp)
#target_link_libraries(grammer_test ${catkin_LIBRARIES})
#target_link_libraries(grammer_test ${LIB_DIR_CONNECT})
#add_dependencies(grammer_test sense_glove_test_generate_messages_cpp)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


