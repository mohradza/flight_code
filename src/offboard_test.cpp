/*
 * @file small_object_avoidance.cpp
 * @brief Full implementation of OF (fourier residual approach) steering controller.
 *        Takeoff in manual control mode, then transfer control over to OFFBOARD
 *        in position control mode. This node takes in yaw rate commands from 
 *        the small object node (small_object_avoidance.py)
 */
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Float32.h>
#include <object_avoidance/YawRateCmdMsg.h>
#include <flight_code/ControllerOutMsg.h>
#include <flight_code/YawAngleMsg.h>
#include <flight_code/DSwitchMsg.h>

// State Callback
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Pose Callback
// Quaternion to Euler Transformation for yaw angle relative to North
flight_code::YawAngleMsg yaw_angle_msg;
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
    yaw_angle_msg.yaw_angle = std::atan2(t3, t4);
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

// Yaw rate command callback
object_avoidance::YawRateCmdMsg yaw_cmd;
void yaw_cmd_cb(const object_avoidance::YawRateCmdMsg::ConstPtr& msg){
    yaw_cmd = *msg;
}


// MAIN LOOP
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber RC_in_sub = nh.subscribe<mavros_msgs::RCIn>
            ("/mavros/rc/in", 10, RCin_cb);
    ros::Subscriber yaw_cmd_sub = nh.subscribe<object_avoidance::YawRateCmdMsg>
            ("/controller_out/yaw_rate_cmd", 10, yaw_cmd_cb);

    // Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher yaw_angle_pub = nh.advertise<flight_code::YawAngleMsg>
            ("yaw_angle",10);
    ros::Publisher dswitch_pub = nh.advertise<flight_code::DSwitchMsg>
            ("dswitch",10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Create poses
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    geometry_msgs::PoseStamped forward_pose;
    geometry_msgs::PoseStamped safety_stop_pose;
    // Set the home and takeoff pose
    forward_pose.pose.position.x = 0.0;
    forward_pose.pose.position.y = 0.0;
    forward_pose.pose.position.z = 1.0;
    
    
    // Create velocity commands
    geometry_msgs::TwistStamped velocity;
    velocity.twist.linear.x = 0.0;
    velocity.twist.linear.y = 0.0;
    velocity.twist.linear.z = 0.0;

    // Desired current forward speed
    float v = .75; 

    // Wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Start timer
    ros::Time last_request = ros::Time::now();

    bool arm_switch = false;
    bool yaw_switch1 = false;
    bool yaw_switch2 = false;
    bool Dswitch = false;
    bool Dswitch_out = true;
    bool get_pose = false;
    bool get_pose_switch = false;
    bool setpoint_pose = true;
    bool safety_alt = false;
    float forward_yaw_angle = 0.0;
    flight_code::DSwitchMsg Dswitch_msg;
    Dswitch_msg.dswitch = 0;


    // MAIN CONTROL LOOP
    while(ros::ok()){
        // Start off in Manual flight mode until the desired initial position
        // and orientation are achieved. Control is then transitioned to
        // position hold in offboard mode, followed by velocity control
        // which incorproates the yaw rate commands created by OF_controller.py

        // Wait for system to arm, provide some feedback to the screen
        if (!current_state.armed){
	    ROS_INFO_THROTTLE(1,"Waiting for arm...");
	} else {
            arm_switch = true;
        }
        
        // This loop initializes the position / orientation tracking
        // and sets the setpoint mode to pose.
//        if(arm_switch && current_state.mode == "STABILIZED"){
//            ROS_INFO_THROTTLE(2, "System is Armed and in Stabilized Flight Mode");
//            setpoint_pose = true;
//            forward_pose = current_pose;
//            pose.header.stamp = ros::Time::now();
//            pose.pose.position.x = forward_pose.pose.position.x;
//            pose.pose.position.y = forward_pose.pose.position.y;
//            pose.pose.position.z = forward_pose.pose.position.z;
//    
//            pose.pose.orientation.x = forward_pose.pose.orientation.x;
//            pose.pose.orientation.y = forward_pose.pose.orientation.y;
//            pose.pose.orientation.z = forward_pose.pose.orientation.z;
//            pose.pose.orientation.w = forward_pose.pose.orientation.w;
//        }

        // Check D-Switch Value for internal mode switching
        if(Arr[5] < 1500 || safety_alt){
            ROS_INFO_THROTTLE(10,"DSwitch is down.");
            Dswitch = false;
            yaw_switch1 = false;
            yaw_switch2 = false;
            Dswitch_msg.dswitch = 0;
            if (!get_pose_switch){
            	get_pose = false;
            }
        } else if (Arr[5] >= 1500){
            Dswitch = true;
            ROS_INFO_THROTTLE(10,"Dswitch is up.");
	    Dswitch_msg.dswitch = 1;
        }

        // After transition to offboard mode from TX,
        // stay in position hold mode until D-switch flip
        if(current_state.mode == "OFFBOARD" && !Dswitch){
            ROS_INFO_THROTTLE(2, "System is Armed and in OFFBOARD Mode: Position Hold.");
            if (!get_pose){
               setpoint_pose = true;
               forward_pose = current_pose;
                
               pose.pose.position.x = current_pose.pose.position.x;
               pose.pose.position.y = current_pose.pose.position.y;
               pose.pose.position.z = current_pose.pose.position.z;

               pose.pose.orientation.x = current_pose.pose.orientation.x;
               pose.pose.orientation.y = current_pose.pose.orientation.y;
               pose.pose.orientation.z = current_pose.pose.orientation.z;
               pose.pose.orientation.w = current_pose.pose.orientation.w;
                if (!get_pose_switch){
                    get_pose = true;
                    get_pose_switch = true;
                }
            }

        }


        // Create safety net for +z   
        if (current_pose.pose.position.z > 3.1){
            safety_alt = true;
        }      
        
        if (safety_alt){
           ROS_INFO_THROTTLE(1, "MAX ALTITUDE REACHED, PLEASE SWITCH TO MANUAL CONTROL AND LAND");
        }
      

        // Publish setpoints and record status for MATLAB        
        if(setpoint_pose){
            pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(pose);
            ROS_INFO_THROTTLE(2,"Publishing setpoint pose data...");
        } else {
            velocity.header.stamp = ros::Time::now();
            local_vel_pub.publish(velocity);            
            ROS_INFO_THROTTLE(2,"Publishing setpoint vel data...");
        }
        
        // Publish Dswitch status for easier plotting 
        dswitch_pub.publish(Dswitch_msg);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
