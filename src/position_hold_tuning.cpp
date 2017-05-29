
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

    // Set up Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local/pose", 10, pose_cb);
    // Set up Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher record_data_pub = nh.advertise<std_msgs::Bool>
            ("MATLAB/record",10);
    // Set up Service Clients
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Create poses
    // pose: main variable used for sending pose messages
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    // ** Should probably add some initial orientation ** //


    geometry_msgs::PoseStamped home_pose;
    geometry_msgs::PoseStamped takeoff_pose;

    geometry_msgs::PoseStamped landing_pose;

    // Record Data?
    std_msgs::Bool record_data;
    record_data.data = false;

    // Wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Set the home and takeoff pose
    home_pose = current_pose;
    landing_pose = current_pose;
    landing_pose.pose.position.z = home_pose.pose.position.z - .015;

    takeoff_pose = home_pose;
    takeoff_pose.pose.position.z = 1.0;

    // Send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(home_pose);
        ros::spinOnce();
        rate.sleep();
    }

    // Set up control modes and arm/disarm commands
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;    

    // Start timer and choose initial setpoint type
    ros::Time last_request = ros::Time::now();

    // Initialize first pose command
    pose = home_pose;

    // There has to be a better way to do this...
    bool armed = false;
    bool takeoff = false;
    bool man1 = false;
    bool man2 = false;
    bool landed = false;
    bool arm_switch = false;
    bool offb = false;
    bool flight_ready = false;
    bool sitl = false;

    // MAIN CONTROL LOOP
    while(ros::ok()){

        // Check to see if sitl demo
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
          } // End of Sitl Switch

        // Are we ready to take off?
        if(offb && armed && !flight_ready){
            flight_ready = true;
            ROS_INFO("Armed and in offboard control mode");
        }

        // Takeoff
        if(flight_ready && !man1){
            if(ros::Time::now() - last_request < ros::Duration(2.0)){
                pose = home_pose;
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Sending Pose: takeoff_pose");
                pose = takeoff_pose;
                pose.header.stamp = ros::Time::now();
                man1 = true;
                last_request = ros::Time::now();
                ROS_INFO("Recording");
                record_data.data = true;
            }
        }

        // Hold takeoff position
        if(man1 && !man2){
            if(ros::Time::now() - last_request < ros::Duration(20.0)){
                pose.header.stamp = ros::Time::now();
            } else {
              ROS_INFO("Sending Pose: landing_pose");
              pose = landing_pose;
              pose.header.stamp = ros::Time::now();
              man2 = true;
              last_request = ros::Time::now();
              record_data.data = false;
            }
        }

        // Land
        if(man2 && !landed){
            if(ros::Time::now() - last_request < ros::Duration(3.0)){
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Disarming system...");
                pose.header.stamp = ros::Time::now();
                landed = true;
                last_request = ros::Time::now();
            }
        }
        
        // Publish setpoints and record status
        local_pos_pub.publish(pose);
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
