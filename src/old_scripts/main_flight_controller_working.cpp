
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
#include <flight_code/ControllerOutMsg.h>
#include <Eigen/Geometry>
#include <math.h>


// State Callback
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Pose Calback
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
            ("yaw_out",10);
    ros::Publisher control_pub = nh.advertise<flight_code::ControllerOutMsg>
            ("controller_out",10);
    
    // Service Clients
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Create poses
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    geometry_msgs::PoseStamped home_pose;
    geometry_msgs::PoseStamped takeoff_pose;
    geometry_msgs::PoseStamped landing_pose;

    // Set the home and takeoff pose
    home_pose.pose.position.x = -2.5;
    home_pose.pose.position.y = 0.0;
    home_pose.pose.position.z = 0.0;
    
 
    takeoff_pose = home_pose;
    takeoff_pose.pose.position.z = .7;
    takeoff_pose.pose.orientation.x = 0.0;
    takeoff_pose.pose.orientation.y = 0.0;
    takeoff_pose.pose.orientation.z = 0.0;
    takeoff_pose.pose.orientation.w = -1.0;

    // Create velocity commands
    geometry_msgs::TwistStamped velocity;
    velocity.twist.linear.x = 0.0;
    velocity.twist.linear.y = 0.0;
    velocity.twist.linear.z = 0.0;

    geometry_msgs::TwistStamped stop_vel;
    stop_vel = velocity;

    geometry_msgs::TwistStamped vel_pos_x;
    vel_pos_x = velocity;
    vel_pos_x.twist.linear.x = .5;
    vel_pos_x.twist.linear.y = 0.0;
    vel_pos_x.twist.linear.z = 0.0;

    std_msgs::Float32 v;
    v.data = .75;

    geometry_msgs::TwistStamped vel_neg_x;
    vel_neg_x = velocity;
    vel_neg_x.twist.linear.x = -0.5;

    // Add control message
    flight_code::ControllerOutMsg ctrl_msg;
    ctrl_msg.yaw_rate_cmd = 0.0;
    ctrl_msg.x_vel_body = 0.0;
    ctrl_msg.y_vel_body = 0.0;

    // Setpoint type
    int setpoint_type = 0;
    int setpoint_pose = 0;
    int setpoint_att = 1;
    int setpoint_vel = 2;

    // Send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(home_pose);
        ros::spinOnce();
        rate.sleep();
    }

    // Set up control mode and arm/disarm commands
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;

    // Start timer and choose initial setpoint type
    ros::Time last_request = ros::Time::now();
    setpoint_type = setpoint_pose;

    // Initialize first pose command
    pose = home_pose;


    // There has to be a better way to do this...
    bool armed = false;
    bool takeoff = false;
    bool man1 = false;
    bool man2 = false;
    bool man3 = false;
    bool man4 = false;
    bool man5 = false;
    bool man6 = false;
    bool landed = false;
    bool arm_switch = false;
    bool offb = false;
    bool flight_ready = false;
    bool sitl = false;
    bool Dswitch = false;
    bool Dswitch_out = true;
    bool safety_switch = false;

    // MAIN CONTROL LOOP
    while(ros::ok()){
        // Start of SITL switch
        // If sitl = true, arm and set mode to offboard
        // If sitl = false, arming and mode switches are done by TX
        if(sitl){
            if( current_state.mode != "OFFBOARD" &&
               (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                   offb_set_mode.response.success){
                    ROS_INFO("Offboard enabled");
                    offb = true;
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed &&
                   (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) &&
                       arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                        armed = true;
                    }
                    last_request = ros::Time::now();
                }
            }
        // Else: wait for TX commands
        } else {
            // Wait for system to arm
            if(!current_state.armed){
                if(ros::Time::now() - last_request > ros::Duration(1.0)){
                    ROS_INFO("System not armed, waiting for arm...");
                    armed = false;
                    arm_switch = false;
                    last_request = ros::Time::now();
                }
             } else {
                 if(!arm_switch){
                     armed = true;
                     arm_switch = true;
                     ROS_INFO("System has been armed, waiting for offboard mode...");
                 }
             }
            // Wait for Offboard mode to start flight test
            if(!offb && armed && current_state.mode == "OFFBOARD"){
                offb = true;
                ROS_INFO("Flight Mode changed to OFFBOARD, starting offboard control");
                ROS_INFO("Sending Pose: home_pose");
                last_request = ros::Time::now();
            } else {
                if(!offb && (ros::Time::now() - last_request > ros::Duration(2.0))){
                    ROS_INFO("Waiting for offboard mode...");
                    last_request = ros::Time::now();
                }
            }
        } // End of SITL switch

        // Are we ready to take off?
        if(offb && armed && !flight_ready){
            flight_ready = true;
            ROS_INFO("Armed and in offboard control mode");
            pose.pose.position.x = -2.5;
            pose.pose.position.y = 0.0;
            pose.pose.position.z = 0.0;
        }
        
        // Takeoff
        if(flight_ready && !man1){
            if(ros::Time::now() - last_request < ros::Duration(1.0)){
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Moving to takeoff");
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = -2.5;
                pose.pose.position.y = 0.0;
                pose.pose.position.z = .7;
                man1 = true;
                last_request = ros::Time::now();
            }
        }

        // Move to takeoff
        if(man1 && !takeoff){
            if(ros::Time::now() - last_request < ros::Duration(3.0)){
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Moving to maneuver start pose");
                pose.header.stamp = ros::Time::now();
                takeoff = true;
                last_request = ros::Time::now();
            }
        }

        // Check D-Switch Value for internal mode switching
        if(Arr[5] < 1500){
            Dswitch = false;
        } else if ((Arr[5] >= 1500) && (Dswitch_out)){
            Dswitch = true;
            // This handles the time difference between
            // takeoff and entering forward motion
            last_request = ros::Time::now();
            Dswitch_out = false;
        }
        // Provide system feedback during waiting period
        if(takeoff && !man2 && !Dswitch){
            ROS_INFO_THROTTLE(2, "Waiting for D-Switch flip to begin forward motion");
        }

        // Wait for D switch flip to begin commanding velocities
        if(takeoff && !man2 && Dswitch){
            if(ros::Time::now() - last_request < ros::Duration(.5)){
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Starting Velocity Maneuver: Stop Velocity");
                setpoint_type = setpoint_vel;
                velocity = vel_pos_x;
                velocity.header.stamp = ros::Time::now();
                man2 = true;
                man3 = true;
                last_request = ros::Time::now();
            }
        }
 
        // Stop for 1 second, then start forward motion
        // Start incorporating yaw commands to to the velocity message
        //if(man2 && !man3){
        //    if(ros::Time::now() - last_request < ros::Duration(.5)){
        //        velocity.header.stamp = ros::Time::now();
        //    } else {
        //        ROS_INFO("Velocity Maneuver: Positive Velocity .5 m/s");
        //        velocity = vel_pos_x;
        //        velocity.header.stamp = ros::Time::now();
        //        man3 = true;
        //        last_request = ros::Time::now();
        //    }
       // }

        // Move forward for 10 seconds
        if(man3 && !man4){
            // Continue forward motion until safety limits are reached
            if(current_pose.pose.position.x > 1.0){
                safety_switch = true;
            }
         
            if(!safety_switch){
                velocity.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Velocity Maneuver: Stop Velocity");
                velocity = stop_vel;
                velocity.header.stamp = ros::Time::now();
                man4 = true;
                last_request = ros::Time::now();
            }
        }

        // Stop the velocity, then start holding  current pose
        if(man4 && !man5){
            if(ros::Time::now() - last_request < ros::Duration(1.0)){
                velocity.header.stamp = ros::Time::now(); 
            } else {
                ROS_INFO("Holding current position");
                setpoint_type = setpoint_pose;
                pose = current_pose;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = -1.0;
                pose.header.stamp = ros::Time::now();
                man5 = true;
                last_request = ros::Time::now();
            }
        }
        
        // Hold current pose then land
        if(man5 && !man6){
            if(ros::Time::now() - last_request < ros::Duration(2.0)){
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Landing...");
                pose.pose.position.z = -.15;
                pose.header.stamp = ros::Time::now();
                man6 = true;
                last_request = ros::Time::now();
            }
        }


        // Land
        if(man6 && !landed){
            if(ros::Time::now() - last_request < ros::Duration(5.0)){
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Landing...");
                landed = true;
                last_request = ros::Time::now();
            }
        }
        
        // If we are moving forward, start publishing yaw commands
      //  if (Dswitch && !safety_switch && false){
        //    ctrl_msg.yaw_rate_cmd = yaw_cmd.yaw_rate_cmd;
          //  ctrl_msg.x_vel_body = v.data*cos(yaw.data);
      //      ctrl_msg.y_vel_body = v.data*sin(yaw.data);
            
            //velocity.twist.angular.z = yaw_cmd.yaw_rate_cmd;
            //velocity.twist.linear.x = v.data*cos(yaw.data);
            //velocity.twist.linear.y = v.data*sin(yaw.data);
    //    }

   //     yaw_angle_pub.publish(yaw);
   //     control_pub.publish(ctrl_msg);
 
        // Publish setpoints and record status for MATLAB        
        if(setpoint_type == setpoint_pose){
            local_pos_pub.publish(pose);
        } else if(setpoint_type == setpoint_att){
           // local_att_pub.publish(attitude);
        } else if(setpoint_type == setpoint_vel){
            local_vel_pub.publish(velocity);            
        } 

        // Disarm the system if not done manually upon landing;
        if(landed && current_state.armed){
            if(arming_client.call(disarm_cmd) && disarm_cmd.response.success){
                ROS_INFO("Vehicle disarmed");
                return 0;
             }
        } else if(landed && armed && !current_state.armed){
            ROS_INFO("System disarmed manually, exiting program...");
            return 0;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
