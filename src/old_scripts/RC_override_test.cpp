
/**
 * @file takeoff_hold_test.cpp
 * @brief Adaptation of the offb_node.cpp script. Commands the
 *        vehicle to a height of 1.5m above the local home position
 *        after OFFBOARD mode has been set and the system has been armed.
 * @flight_tested: yes
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int Arr[12];
mavros_msgs::RCIn current_RCin;
void RCin_cb(const mavros_msgs::RCIn::ConstPtr& msg){
    current_RCin = *msg;
    for(int i = 0; i < 12; i++){
        Arr[i] = msg->channels[i];
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Set up Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
//    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
//            ("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber RC_in_sub = nh.subscribe<mavros_msgs::RCIn>
            ("/mavros/rc/in", 10, RCin_cb);
    
    // Set up Publishers
//    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
//            ("mavros/setpoint_position/local", 10);
//    ros::Publisher record_data_pub = nh.advertise<std_msgs::Bool>
//            ("MATLAB/record", 10);
    ros::Publisher RC_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 10);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::OverrideRCIn RC_override_msg;
//    std_msgs::UInt16 signals[12];    
    // MAIN CONTROL LOOP
    while(ros::ok()){

        // Read RCin
        // Handled by callback function
        
        // Remap message to fit OverrideRCin message type
        for(int i = 0; i < 8; i++){
           RC_override_msg.channels[i]  = Arr[i];
        }   

        // Remap mode channel (9) to channel 8 (unused)
        // OverrideRCin only accepts 8 channels currently
        if ((Arr[8] > 2000) || (Arr[8] < 1000)){
            // Do Nothing, bad signal
        } else {
           RC_override_msg.channels[7] = Arr[8];
        }

        // Publish RCin to RC override
        RC_override_pub.publish(RC_override_msg);
        
        ROS_INFO_THROTTLE(5,"Publishing OverrideRCin Messages..");


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
