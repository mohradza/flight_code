
/**
 * @file x_vel_step_test.cpp
 * @brief Adaptation of the offb_node.cpp script. Commands the
 *        vehicle to a takeoff position, then a maneuver start position,
 *        then it commands a step velocity input, halts, and returns to land.
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
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
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher record_data_pub = nh.advertise<std_msgs::Bool>
            ("MATLAB/record",10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
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

    geometry_msgs::PoseStamped man_start_pose;
    man_start_pose.pose.position.x = 0.0;
    man_start_pose.pose.position.y = 1.5;
    man_start_pose.pose.position.z = 1.0;

    geometry_msgs::PoseStamped landing_pose;


    // Create velocity commands
    geometry_msgs::TwistStamped velocity;
    velocity.twist.linear.x = 0.0;
    velocity.twist.linear.y = 0.0;
    velocity.twist.linear.z = 0.0;

    geometry_msgs::TwistStamped stop_vel;
    stop_vel = velocity;

    geometry_msgs::TwistStamped vel_pos_x;
    vel_pos_x = velocity;
    vel_pos_x.twist.linear.x = 0.25;

    geometry_msgs::TwistStamped vel_pos_y;
    vel_pos_y = velocity;
    vel_pos_y.twist.linear.y = 0.25;

    geometry_msgs::TwistStamped vel_neg_x;
    vel_neg_x = velocity;
    vel_neg_x.twist.linear.x = -0.25;

    geometry_msgs::TwistStamped vel_neg_y;
    vel_neg_y = velocity;
    vel_neg_y.twist.linear.y = -.5;

    // Record Data?
    std_msgs::Bool record_data;
    record_data.data = false;


    // Setpoint type
    int setpoint_type = 0;
    int setpoint_pose = 0;
    int setpoint_att = 1;
    int setpoint_vel = 2;


    // Wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Set the home and takeoff pose
    home_pose = current_pose;
 
    takeoff_pose = home_pose;
    takeoff_pose.pose.position.z = 1.0;
    takeoff_pose.pose.orientation.x = 0.0;
    takeoff_pose.pose.orientation.y = 0.0;
    takeoff_pose.pose.orientation.z = 0.0;
    takeoff_pose.pose.orientation.w = 1.0;

    landing_pose = takeoff_pose;
    landing_pose.pose.position.z = home_pose.pose.position.z - .015;

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
        }

        
        // Takeoff
        if(flight_ready && !man1){
            if(ros::Time::now() - last_request < ros::Duration(3.0)){
                pose = home_pose;
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Moving to takeoff");
                pose = takeoff_pose;
                pose.header.stamp = ros::Time::now();
                man1 = true;
                last_request = ros::Time::now();
            }
        }

        // Move to takeoff
        if(man1 && !takeoff){
            if(ros::Time::now() - last_request < ros::Duration(5.0)){
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Moving to maneuver start pose");
                pose = man_start_pose;
                pose.header.stamp = ros::Time::now();
                takeoff = true;
                last_request = ros::Time::now();
            }
        }


        // Move to first maneuver
        if(takeoff && !man2){
            if(ros::Time::now() - last_request < ros::Duration(5.0)){
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Starting Velocity Maneuver: Stop Velocity");
                setpoint_type = setpoint_vel;
                velocity = stop_vel;
                velocity.header.stamp = ros::Time::now();
                record_data.data =true;
                man2 = true;
                last_request = ros::Time::now();
            }
        }


        // Hold a Zero Velocity to initialize
        if(man2 && !man3){
            if(ros::Time::now() - last_request < ros::Duration(8.0)){
                velocity = stop_vel;
                velocity.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Velocity Maneuver: Positive Velocity .2 m/s");
                velocity = vel_neg_y;
                velocity.header.stamp = ros::Time::now();
                man3 = true;
                last_request = ros::Time::now();
            }
        }
        
        // Move forwards for 8 seconds
        if(man3 && !man4){
            if(ros::Time::now() - last_request < ros::Duration(4.0)){
                velocity = vel_neg_y;
                velocity.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Velocity Maneuver: Stop Velocity");
                velocity = stop_vel;
                velocity.header.stamp = ros::Time::now();
                man4 = true;
                last_request = ros::Time::now();
            }
        }

        // Stop the velocity
        if(man4 && !man5){
            if(ros::Time::now() - last_request < ros::Duration(8.0)){
                velocity = stop_vel;
                velocity.header.stamp = ros::Time::now(); 
            } else {
                ROS_INFO("Returning to Takeoff Position");
                record_data.data = false;
                setpoint_type = setpoint_pose;
                pose = takeoff_pose;
                pose.header.stamp = ros::Time::now();
                man5 = true;
                last_request = ros::Time::now();
            }
        }
        
        // Return to Takeoff
        if(man5 && !man6){
            if(ros::Time::now() - last_request < ros::Duration(3.0)){
                pose = takeoff_pose;
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Landing...");
                pose = landing_pose;
                pose.header.stamp = ros::Time::now();
                man6 = true;
                last_request = ros::Time::now();
            }
        }


        // Land
        if(man6 && !landed){
            if(ros::Time::now() - last_request < ros::Duration(3.0)){
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Landing...");
                landed = true;
                last_request = ros::Time::now();
            }
        }

        // Publish setpoints and record status for MATLAB        
        if(setpoint_type == setpoint_pose){
            local_pos_pub.publish(pose);
        } else if(setpoint_type == setpoint_att){
           // local_att_pub.publish(attitude);
        } else if(setpoint_type == setpoint_vel){
            local_vel_pub.publish(velocity);            
        } 
        record_data_pub.publish(record_data);

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
