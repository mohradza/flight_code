
/**
 * @file x_roll_test.cpp
 * @brief Adaptation of the offb_node.cpp script. Commands a +10 degree roll constantly
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local/pose", 10, pose_cb);
    // Publishers
    ros::Publisher pub_att = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);

    ros::Publisher pub_thr = nh.advertise<std_msgs::Float64>
            ("mavros/setpoint_attitude/att_throttle",10);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Create attitude in quaternions based on 3-2-1 rotation
    geometry_msgs::PoseStamped pos_roll;
    pos_roll.header.stamp = ros::Time::now();
    pos_roll.header.seq = 0;
    pos_roll.header.frame_id = 1;
    pos_roll.pose.position.x = 0.0; 
    pos_roll.pose.position.y = 0.0;
    pos_roll.pose.position.z = 0.0;

    // Tshi should correspond to a +10 degree roll angle
    pos_roll.pose.orientation.x = 0.1;
    pos_roll.pose.orientation.y = 0.0;
    pos_roll.pose.orientation.z = 0.0;
    pos_roll.pose.orientation.w = 0.995;

    // Throttle command
    std_msgs::Float64 throttle;

    // Wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Set the home and takeoff pose
    geometry_msgs::PoseStamped home_pose;
    home_pose = current_pose;
    
    // Send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pub_att.publish(pos_roll);
        ros::spinOnce();
        rate.sleep();
    }

    // Start timer and choose initial setpoint type
    ros::Time last_request = ros::Time::now();

    int count = 1;
    // MAIN CONTROL LOOP
    while(ros::ok()){

       pos_roll.header.stamp = ros::Time::now();
       pos_roll.header.seq=count;
       pos_roll.header.frame_id = 1;

       //Create throttle command message
       //cmd_thr.data = throttle;
       
       pub_att.publish(pos_roll);
  //     pub_thr.publish(cmd_thr);
       count++;
       ros::spinOnce();
       rate.sleep();

       ROS_INFO_THROTTLE(3,"Sending quaternion commands");
    }

    return 0;
}
