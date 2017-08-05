
/**
 * @file x_vel_step_test.cpp
 * @brief Adaptation of the offb_node.cpp script. Commands the
 *        vehicle to a takeoff position, then a maneuver start position,
 *        then it commands a step velocity input, halts, and returns to land.
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
#include <flight_code/AllDataOutMsg.h>
#include <small_object/OpticFlowMsg.h>
#include <Eigen/Geometry>
#include <math.h>

// Vicon Pose Callback
geometry_msgs::PoseStamped vicon_pose;
void vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    vicon_pose = *msg;
    ROS_INFO_THROTTLE(2,"Pose cb");
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

// Pose Callback
std_msgs::Float32 yaw;
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
    double x = current_pose.pose.orientation.x;
    double y = current_pose.pose.orientation.y;
    double z = current_pose.pose.orientation.z;
    double w = current_pose.pose.orientation.w;
    double ysqr = y * y;

	// roll (x-axis rotation)
	//double t0 = +2.0 * (q.w() * q.x() + q.y() * q.z());
	//double t1 = +1.0 - 2.0 * (q.x() * q.x() + ysqr);
	//roll = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	//double t2 = +2.0 * (q.w() * q.y() - q.z() * q.x());
	//t2 = t2 > 1.0 ? 1.0 : t2;
	//t2 = t2 < -1.0 ? -1.0 : t2;
	//pitch = std::asin(t2);

	// yaw (z-axis rotation)
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (ysqr + z * z);  
    yaw.data = std::atan2(t3, t4);
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber RC_in_sub = nh.subscribe<mavros_msgs::RCIn>
            ("/mavros/rc/in", 10, RCin_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, pose_cb); 
    ros::Subscriber vicon_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/ohrad_quad/pose", 10, vicon_pose_cb);
    ros::Subscriber vicon_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/vrpn_client_node/ohrad_quad/twist", 10, vicon_vel_cb);
    ros::Subscriber vicon_accel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/vrpn_client_node/ohrad_quad/accel", 10, vicon_accel_cb);
    ros::Subscriber OF_sub = nh.subscribe<small_object::OpticFlowMsg>
            ("/optic_flow/optic_flow", 10, OF_cb);
    ros::Subscriber setpoint_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10, cmd_vel_cb);
    ros::Subscriber dswitch_sub = nh.subscribe<flight_code::DSwitchMsg>
            ("/dswitch", 10, dswitch_cb);

    // Publishers
    ros::Publisher full_data_pub = nh.advertise<flight_code::AllDataOutMsg>
            ("AllDataOut", 10);
    
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Start timer and choose initial setpoint type
    ros::Time last_request = ros::Time::now();

    // Create full data message
    flight_code::AllDataOutMsg full_msg;

    // MAIN CONTROL LOOP
    while(ros::ok()){

        // Create full message with all topics
        full_msg.header.stamp = ros::Time::now();
        full_msg.yaw_rate_cmd = cmd_vels.twist.angular.z;
        full_msg.x_vel_cmd = cmd_vels.twist.linear.x;
        full_msg.y_vel_cmd = cmd_vels.twist.linear.y;

        full_msg.yaw_angle = yaw.data;

        full_msg.OF_meas = OF_data.Qdot_meas;
        full_msg.OF_SF = OF_data.Qdot_SF;

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
