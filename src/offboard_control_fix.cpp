
/**
 * @file y_vel_step_test.cpp
 * @brief Adaptation of the offb_node.cpp script. Commands the
 *        vehicle to a takeoff position, then a maneuver start position,
 *        then it commands a step velocity input, halts, and returns to land.
 */

#include <ros/ros.h>
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

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher setpoint_type_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint/type", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
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
    man_start_pose.pose.position.y = 0.0;
    man_start_pose.pose.position.z = 1.5;

    geometry_msgs::PoseStamped landing_pose;

    // This is currently used to start recording data in MATLAB
    geometry_msgs::PoseStamped setpoint_type_msg;    
    setpoint_type_msg = pose;


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
    landing_pose = current_pose;
    landing_pose.pose.position.z = home_pose.pose.position.z - .05;

    takeoff_pose = home_pose;
    takeoff_pose.pose.position.z = 1.5;

    // Send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(home_pose);
        ros::spinOnce();
        rate.sleep();
    }

    // Set up control modes and arm/disarm commands
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode RTL_set_mode;
    RTL_set_mode.request.custom_mode = "AUTO_RTL";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;    

    // Start timer and choose initial setpoint type
    ros::Time last_request = ros::Time::now();
    setpoint_type = setpoint_pose;
    setpoint_type_msg.pose.position.x = 0.0; 

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
    bool landed = false;
    bool arm_switch = false;
    bool offb = false;
    bool RTL = false;
    bool POSCTL = false;

    // MAIN CONTROL LOOP
    while(ros::ok()){

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

        // Takeoff
        if(offb && !man1){
            if(ros::Time::now() - last_request < ros::Duration(5.0)){
                pose = home_pose;
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Sending Pose: takeoff_pose");
                pose = takeoff_pose;
                pose.header.stamp = ros::Time::now();
                man1 = true;
                last_request = ros::Time::now();
            }
        }

        // Hold takeoff position
        if(man1 && !man2){
            if(ros::Time::now() - last_request < ros::Duration(10.0)){
                pose.header.stamp = ros::Time::now();
            } else {
              ROS_INFO("Sending Pose: landing_pose");
              pose = landing_pose;
              pose.header.stamp = ros::Time::now();
              man2 = true;
              last_request = ros::Time::now();
            }
        }

        // Land
        if(man2 && !landed){
            if(ros::Time::now() - last_request < ros::Duration(5.0)){
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Disarming system...");
                pose.header.stamp = ros::Time::now();
                landed = true;
                last_request = ros::Time::now();
            }
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


        // If Offboard mode is disconnected for some reason, resort to RTL
        if(offb && (current_state.mode != "OFFBOARD")){
            if(!POSCTL && current_state.mode == "POSCTL"){
                ROS_INFO("Flight Mode changed to POSCTL");
                POSCTL = true;
            } else if (!RTL && current_state.mode == "AUTO_RTL"){
                ROS_INFO("Flight Mode changed to AUTO_RTL");
                RTL = true;
            } else {
                if(ros::Time::now() - last_request > ros::Duration(1.0)){
                    if(!RTL && set_mode_client.call(RTL_set_mode) 
                       && RTL_set_mode.response.success){
                        ROS_INFO("Flight mode changed to RTL");
                        RTL = true;
                    }
                    last_request = ros::Time::now();      
                }
            }         
        }

        // Always publish the setpoint velocity topic
        setpoint_type_msg.header.stamp = ros::Time::now();
        setpoint_type_pub.publish(setpoint_type_msg);

        // Publish setpoints
        if(setpoint_type == setpoint_pose){
            local_pos_pub.publish(pose);
        } else if(setpoint_type == setpoint_att){
           // local_att_pub.publish(attitude);
        } else if(setpoint_type == setpoint_vel){
//            local_vel_pub.publish(velocity);
        }
         
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
