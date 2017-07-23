
/**
 * @file x_vel_step_test.cpp
 * @brief Adaptation of the offb_node.cpp script. Commands the
 *        vehicle to a takeoff position, then a maneuver start position,
 *        then it commands a step velocity input, halts, and returns to land.
 */
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float32.h>


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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    // Publishers
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);

    ros::Publisher yaw_angle_pub = nh.advertise<std_msgs::Float32>
            ("yaw_angle",10);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    
    // Create velocity commands
    geometry_msgs::TwistStamped velocity;
    velocity.twist.linear.x = 0.0;
    velocity.twist.linear.y = 0.0;
    velocity.twist.linear.z = 0.0;

    geometry_msgs::TwistStamped stop_vel;
    stop_vel = velocity;

    geometry_msgs::TwistStamped vel_pos_x;
    vel_pos_x = velocity;
    float psi = 3.0;
    float v = .5; 
    vel_pos_x.twist.linear.x = v*cos(psi);
    vel_pos_x.twist.linear.y = -v*sin(psi);

    geometry_msgs::TwistStamped vel_neg_x;
    vel_neg_x = velocity;
    vel_neg_x.twist.linear.x = -0.5;


    // Wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Start timer
    ros::Time last_request = ros::Time::now();

    // Initialize first pose command
    velocity = vel_pos_x;
    bool arm_switch = false;
    bool yaw_switch1 = false;
    bool yaw_switch2 = false;
    float yaw_angle = 0.0;
    // MAIN CONTROL LOOP
    while(ros::ok()){
        // Just need to publish a forward velocity
        // We will be switching from position hold to offboard mode
        if (!current_state.armed && !yaw_switch1){
	    ROS_INFO_THROTTLE(1,"Waiting for arm...");
            velocity.twist.linear.x = 0.0;
            velocity.twist.linear.y = 0.0;
            velocity.twist.linear.z = 0.0;
	} else {
            arm_switch = true;
        }
        
        if(arm_switch && current_state.mode == "STABILIZED" && !yaw_switch1){
            ROS_INFO_THROTTLE(2, "System is Armed and in Stabilized Flight Mode");
            velocity.twist.linear.x = 0.0;
            velocity.twist.linear.y = 0.0;
            velocity.twist.linear.z = 0.0;
        }

        if(current_state.mode == "OFFBOARD" && !yaw_switch2){
            velocity.twist.linear.x = v*cos(yaw.data);
            velocity.twist.linear.y = v*sin(yaw.data);
            velocity.twist.linear.z = 0.0;
            yaw_switch2 = true;
        }
   
        // Publish velocity setpoint      
        local_vel_pub.publish(velocity);
        yaw_angle_pub.publish(yaw);            
          
        if(yaw_switch2){
            ROS_INFO_THROTTLE(5,"Publishing Forward Velocity to /mavros/setpoint_velocity/cmd_vel");
        }
        //  ROS_INFO_THROTTLE(1,"Current yaw angle is: %f",yaw);
        // ROS rate tick
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
