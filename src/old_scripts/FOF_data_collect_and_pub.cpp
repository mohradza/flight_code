
/**
 * @file  FOF_data_collection_and_pub.cpp
 * @brief Gathers all data related to Flow of Flow and
 *           publishes it as on message with a shared header.
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <flight_code/YawRateCmdMsg.h>
#include <flight_code/DSwitchMsg.h>
#include <flight_code/FOFAllDataOutMsg.h>
#include <flight_code/AllDataOutMsg.h>
#include <small_object/OpticFlowMsg.h>
#include <small_object/FOF_and_ResidualMsg.h>
#include <Eigen/Geometry>
#include <math.h>

// Vicon Pose Callback
geometry_msgs::PoseStamped vicon_pose;
void vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    vicon_pose = *msg;
//    ROS_INFO_THROTTLE(2,"Pose cb");
}

// Vicon Velocity Callback
geometry_msgs::TwistStamped vicon_vel;
void vicon_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    vicon_vel = *msg;
}

// Vicon Accel Callback
geometry_msgs::TwistStamped vicon_accel;
void vicon_accel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    vicon_accel = *msg;
}


// RCin callback
int Arr[12];
mavros_msgs::RCIn current_RCin;
void RCin_cb(const mavros_msgs::RCIn::ConstPtr& msg){
    current_RCin = *msg;
    for(int i = 0; i < 12; i++){
        Arr[i] = msg->channels[i];
    }
}

// Velocity command callback
geometry_msgs::TwistStamped cmd_vels;
void cmd_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    cmd_vels = *msg;
}

// Optic Flow Callback
small_object::OpticFlowMsg OF_data;
void OF_cb(const small_object::OpticFlowMsg::ConstPtr& msg){
     OF_data = *msg;
}

// Dswitch callback
flight_code::DSwitchMsg dswitch;
void dswitch_cb(const flight_code::DSwitchMsg::ConstPtr& msg){
     dswitch = *msg;
}

// FOF callback
small_object::FOF_and_ResidualMsg FOF_data_in;
void FOF_cb(const small_object::FOF_and_ResidualMsg::ConstPtr& msg){
    FOF_data_in = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber RC_in_sub = nh.subscribe<mavros_msgs::RCIn>
            ("/mavros/rc/in", 10, RCin_cb);
    ros::Subscriber vicon_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/ohrad_quad/pose", 10, vicon_pose_cb);
    ros::Subscriber vicon_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/vrpn_client_node/ohrad_quad/twist", 10, vicon_vel_cb);
    ros::Subscriber vicon_accel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/vrpn_client_node/ohrad_quad/accel", 10, vicon_accel_cb);
    ros::Subscriber setpoint_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10, cmd_vel_cb);
    ros::Subscriber dswitch_sub = nh.subscribe<flight_code::DSwitchMsg>
            ("/dswitch", 10, dswitch_cb);
    ros::Subscriber FOF_sub = nh.subscribe<small_object::FOF_and_ResidualMsg>
            ("/FOF_data", 10, FOF_cb);
    
    // Publishers
    ros::Publisher full_data_pub = nh.advertise<flight_code::FOFAllDataOutMsg>
            ("FOFAllDataOut", 10);
    
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Start timer and choose initial setpoint type
    ros::Time last_request = ros::Time::now();

    // Create full data message
    flight_code::FOFAllDataOutMsg full_msg;

    // MAIN CONTROL LOOP
    while(ros::ok()){

        // Create full message with all topics
        full_msg.header.stamp = ros::Time::now();

        full_msg.OF_meas = FOF_data_in.Qdot_meas;
        full_msg.FOF_OF_SF = FOF_data_in.FOF_OF_SF;
        full_msg.FOF_yaw_rate_cmd = FOF_data_in.FOF_yaw_rate_cmd;
        full_msg.FR_yaw_rate_cmd = FOF_data_in.FR_yaw_rate_cmd;
        full_msg.FR_OF_SF = FOF_data_in.FR_Qdot_SF;

        full_msg.vicon_position_x = vicon_pose.pose.position.x;
        full_msg.vicon_position_y = vicon_pose.pose.position.y;
        full_msg.vicon_position_z = vicon_pose.pose.position.z;
        full_msg.vicon_orientation_x = vicon_pose.pose.orientation.x;
        full_msg.vicon_orientation_y = vicon_pose.pose.orientation.y;
        full_msg.vicon_orientation_z = vicon_pose.pose.orientation.z;
        full_msg.vicon_orientation_w = vicon_pose.pose.orientation.w;

        full_msg.vicon_vel_x = vicon_vel.twist.linear.x;
        full_msg.vicon_vel_y = vicon_vel.twist.linear.y;
        full_msg.vicon_vel_z = vicon_vel.twist.linear.z;
        full_msg.vicon_angular_z = vicon_vel.twist.angular.z;

        full_msg.vicon_accel_x = vicon_accel.twist.linear.x;
        full_msg.vicon_accel_y = vicon_accel.twist.linear.y;
        full_msg.vicon_accel_z = vicon_accel.twist.linear.z;

        full_msg.dswitch = dswitch.dswitch;

        full_data_pub.publish(full_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
