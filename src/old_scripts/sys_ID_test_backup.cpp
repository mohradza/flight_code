
/**
 * @file takeoff_hold_test.cpp
 * @brief Adaptation of the offb_node.cpp script. Commands the
 *        vehicle to a height of 1.5m above the local home position
 *        after OFFBOARD mode has been set and the system has been armed.
 * @flight_tested: yes
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
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


// RCin Channels:
// 0: Roll, 1: Pitch, 2: Throttle, 3: Yaw, 4: , 5: D-Switch, 6: E-Switch, 7: Knob, 8: Mode switch 

mavros_msgs::RCIn current_RCin;
int Arr[12];
void RCin_cb(const mavros_msgs::RCIn::ConstPtr msg){
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

    // Set up Service Clients
//    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
  //          ("mavros/set_mode");
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Setup timing variable
    ros::Time last_request = ros::Time::now();

    // Wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    // OverrideRCin Channels:
    // 0: Roll, 1: Pitch, 2: Throttle, 3: Yaw, 4:, 5: D-Switch, 6: E-Switch, 7: Mode Switch
    mavros_msgs::OverrideRCIn RC_override_msg;
    mavros_msgs::OverrideRCIn RC_eq;
    bool switch1 = false;
    bool switch2 = false;
    bool switch3 = false;
    bool eq_vals = false; 
    bool dub1 = false;
    bool dub2 = false;
    bool dub3 = false;

    // MAIN CONTROL LOOP
    while(ros::ok()){

        // Read RCin
        // Handled by callback function
	
        // Check D-Switch Value for internal mode switching
        if(Arr[5] < 1500){
            // Do nothing i
            ROS_INFO_THROTTLE(5,"Manual Control...Waiting for D-Switch Flip...");        
        } else if (Arr[5] >= 1500){
            switch1 = true;
        }
        
        
        // Grab current RCin values and set them as the equilibrium RCin values
        // System should already be in Altitude Hold mode
        if(switch1 && !switch2){
            ROS_INFO_THROTTLE(1,"Grabbing  equilibrium values and publishing eq. pose RC vals...");        
            // Grab current RCin values and set them as the equilibrium RCin values
            if(!eq_vals){
                for(int i = 0; i < 8; i++){
                   RC_eq.channels[i] = Arr[i];
                   RC_override_msg.channels[i] = Arr[i];
                }   
                
                // Remap mode channel (9) to channel 8 (unused)
                // OverrideRCin only accepts 8 channels currently
                if ((Arr[8] > 2000) || (Arr[8] < 1000)){
                   // Do Nothing, bad signal
                } else {
                   RC_override_msg.channels[7] = Arr[8];
                }

                eq_vals = true;
                last_request = ros::Time::now();
            } 
             
            if(ros::Time::now() - last_request < ros::Duration(2.0)){
                // Publish RC equilibrium value for 2 second
                ROS_INFO("Publishing eq. vals...");
                RC_override_pub.publish(RC_override_msg);
             } else {
                switch2 = true;
                last_request = ros::Time::now();
             }
           }
 
        // Execute doublet manuever
        if(switch2 && !switch3){
               ROS_INFO_THROTTLE(1, "Executing Doublet Maneuver...");
               // Subtract roll angle: try - 30 PWM
               if(!dub1){
                   if(ros::Time::now() - last_request < ros::Duration(.5)){
                       RC_override_msg.channels[1] = RC_eq.channels[1] - 100;
                       ROS_INFO_THROTTLE(1, "Negative Pitch");
                   } else {
                       dub1 = true;
                       last_request = ros::Time::now();
                   }
               }
                 
               // Subtract roll angle: try + 60 PWM
               if(!dub2 && dub1){
                   if(ros::Time::now() - last_request < ros::Duration(.5)){
                       RC_override_msg.channels[1] = RC_eq.channels[1] + 100;
                       ROS_INFO_THROTTLE(1, "Positive Pitch");
                   } else {
                       dub2= true;
                       last_request = ros::Time::now();
                   }
               }

               // Command equlibrium
               if(!dub3 && dub2){
                   if(ros::Time::now() - last_request < ros::Duration(2.0)){
                       RC_override_msg.channels[1] = RC_eq.channels[1];
                       ROS_INFO_THROTTLE(1, "Equilibrium State");
                   } else {
                       dub3 = true;
                       switch3 = true;
                       switch1 = false;
                   }
               }
         }

         // Return Manual control
         // Command equlibrium
         if(switch3){
             ROS_INFO_THROTTLE(1, "Manual Control Regained...");
             // Remap message to fit OverrideRCin message type
             for(int i = 0; i < 8; i++){
                 RC_override_msg.channels[i] = Arr[i];
             }   

             // Remap mode channel (9) to channel 8 (unused)
             // OverrideRCin only accepts 8 channels currently
             if ((Arr[8] > 2000) || (Arr[8] < 1000)){
                // Do Nothing, bad signal
             } else {
                  RC_override_msg.channels[7] = Arr[8];
             }
         }
         if(switch1){     
             // Publish RC message
             RC_override_pub.publish(RC_override_msg); 
         }
         ros::spinOnce();
         rate.sleep();
    }

    return 0;
}
