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
#include <flight_code/YawRateCmdMsg.h>
#include <flight_code/ControllerOutMsg.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
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

// Yaw rate command callback
flight_code::YawRateCmdMsg yaw_cmd;
void yaw_cmd_cb(const flight_code::YawRateCmdMsg::ConstPtr& msg){
    yaw_cmd  = *msg;
}

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
    ros::Subscriber yaw_cmd_sub = nh.subscribe<flight_code::YawRateCmdMsg>
            ("/controller_out/yaw_rate_cmd", 10, yaw_cmd_cb);

    // Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher yaw_angle_pub = nh.advertise<std_msgs::Float32>
            ("yaw_angle",10);
    ros::Publisher control_pub = nh.advertise<flight_code::ControllerOutMsg>
            ("controller_out",10);
    ros::Publisher dswitch_pub = nh.advertise<std_msgs::Int16>
            ("dswitch",10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Create poses
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    geometry_msgs::PoseStamped forward_pose;

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
    float v = .8; 

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
    bool setpoint_pose = true;
    float forward_yaw_angle = 0.0;
    std_msgs::Int16 Dswitch_msg;
    Dswitch_msg.data = 0;


    // MAIN CONTROL LOOP
    while(ros::ok()){
        // Just need to publish a forward velocity
        // We will be switching from position hold to offboard mode
        if (!current_state.armed){
	    ROS_INFO_THROTTLE(1,"Waiting for arm...");
	} else {
            arm_switch = true;
        }
        
        if(arm_switch && current_state.mode == "STABILIZED"){
            ROS_INFO_THROTTLE(2, "System is Armed and in Stabilized Flight Mode");
            setpoint_pose = true;
            forward_pose = current_pose;
            pose.pose.position.x = forward_pose.pose.position.x;
            pose.pose.position.y = forward_pose.pose.position.y;
            pose.pose.position.z = forward_pose.pose.position.z;
    
            pose.pose.orientation.x = forward_pose.pose.orientation.x;
            pose.pose.orientation.y = forward_pose.pose.orientation.y;
            pose.pose.orientation.z = forward_pose.pose.orientation.z;
            pose.pose.orientation.w = forward_pose.pose.orientation.w;
        }

        // Check D-Switch Value for internal mode switching
        if(Arr[5] < 1500){
            ROS_INFO_THROTTLE(10,"DSwitch is down.");
            Dswitch = false;
            yaw_switch1 = false;
            yaw_switch2 = false;
            Dswitch_msg.data = 0;
        } else if ((Arr[5] >= 1500) && (Dswitch_out)){
            Dswitch = true;
            ROS_INFO_THROTTLE(10,"Dswitch is up.");
	    Dswitch_msg.data = 1;
        }

        // Stay in position hold mode until D-sswitch flip

        if(current_state.mode == "OFFBOARD" && !Dswitch){
            ROS_INFO_THROTTLE(2, "System is Armed and in OFFBOARD Mode: Position Hold.");
            if (!get_pose){
                forward_pose = current_pose;
                pose.pose.position.x = forward_pose.pose.position.x;
                pose.pose.position.y = forward_pose.pose.position.y;
                pose.pose.position.z = forward_pose.pose.position.z;

                pose.pose.orientation.x = forward_pose.pose.orientation.x;
                pose.pose.orientation.y = forward_pose.pose.orientation.y;
                pose.pose.orientation.z = forward_pose.pose.orientation.z;
                pose.pose.orientation.w = forward_pose.pose.orientation.w;
                get_pose = true;
            }

        }

        // Once the Dswitch is flipped, initiate the switch to velocity mode
        if (Dswitch){
            yaw_switch1 = true;
            get_pose = false;
        }

        // Transition the vehicle to moving forward in velocity mode
        if (yaw_switch1 && !yaw_switch2){
           velocity.twist.linear.x = v*cos(forward_yaw_angle);
           velocity.twist.linear.y = v*sin(forward_yaw_angle);
           velocity.twist.linear.z = 0.0;

           velocity.twist.angular.x = 0.0;
           velocity.twist.angular.y = 0.0;
           velocity.twist.angular.z = 0.0;
           
           setpoint_pose = false;
           yaw_switch2 = true;
        }

        // Once we are moving forward, start taking yaw commands
        if (yaw_switch2){
           //velocity.twist.angular.z = yaw_cmd.yaw_rate_cmd;
           velocity.twist.linear.x = v*cos(yaw.data);
           velocity.twist.linear.y = v*sin(yaw.data);
           velocity.twist.linear.z = 0.0;

        }         

        // Publish setpoints and record status for MATLAB        
        if(setpoint_pose){
            pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(pose);
            ROS_INFO_THROTTLE(2,"Publishing setpoint pose data...");
       // } else if(setpoint_type == setpoint_att){
           // local_att_pub.publish(attitude);
        } else {
            velocity.header.stamp = ros::Time::now();
            local_vel_pub.publish(velocity);            
            ROS_INFO_THROTTLE(2,"Publishing setpoint vel data. twist.angular.z = %f", velocity.twist.angular.z);
        } 
    
        yaw_angle_pub.publish(yaw);            
        dswitch_pub.publish(Dswitch_msg);
        //ROS_INFO_THROTTLE(2,"Current yaw angle is: %f", yaw.data);
        
        //  ROS_INFO_THROTTLE(1,"Current yaw angle is: %f",yaw);
        // ROS rate tick
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
