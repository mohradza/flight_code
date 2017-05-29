#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State currentState; // Create an object to identify the current state of the Quad
void state_callback(const mavros_msgs::State::ConstPtr& msg)
{
	currentState = *msg;

}


geometry_msgs::PoseStamped currentPose; // Get current position of the Quad
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	currentPose = *msg;
	float z_pos = currentPose.pose.position.z;
	ROS_INFO("z pos: %f", z_pos);
	
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "AVL_LeafBlower_Node");
	
	ros::NodeHandle n;

	// ROS Publisher
	ros::Publisher mavros_pos_control_pub = n.advertise<geometry_msgs::PoseStamped>
			("mavros/setpoint_position/local",20);
	ros::Publisher local_vel_pub = n.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 20);

	// ROS Subscriber 
	ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
			("mavros/state",20, state_callback);

	ros::Subscriber local_pos_sub = n.subscribe<geometry_msgs::PoseStamped>
			("mavros/local_position/local",20,pose_callback);

	// ROS Client/Services:
	ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
	ros::ServiceClient mavros_set_mode_client = n.serviceClient<mavros_msgs::SetMode>
			("/mavros/set_mode");
  	mavros_msgs::SetMode set_mode;
  	set_mode.request.custom_mode = "OFFBOARD";

  	bool offboard_commands_enabled = false;
  	bool chk1, chk2,chk3,chk4,chk5,chk6,chk7,chk8,chk9,chk10,chk11;



  	ros::Rate loop_rate(30);

  	geometry_msgs::PoseStamped pos_sp; // object for set point
  	geometry_msgs::PoseStamped home_pose; 
  	home_pose.pose.position.z = currentPose.pose.position.z - 0.2;

  	geometry_msgs::TwistStamped velo_sp;
  	
  	
  	for(int i = 100; ros::ok() && i > 0; --i){
            mavros_pos_control_pub.publish(home_pose);
            ros::spinOnce();
            loop_rate.sleep();
        }

        bool pos_setpoint_flag = true;
        bool armed = false;
        bool arm_switch = false;
        bool offb = false;

    ros::Time start_time = ros::Time::now();
    ros::Time last_request = start_time;
  	while (ros::ok())
  	{
            // Wait for system to arm
            if(!currentState.armed){
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
            if(!offb && armed && currentState.mode == "OFFBOARD"){
                offb = true;
                ROS_INFO("Flight Mode changed to OFFBOARD, starting offboard control");
                ROS_INFO("Sending Pose: home_pose");
                last_request = ros::Time::now();
            } else {
                if(!offb && (ros::Time::now() - last_request > ros::Duration(2.0))){
                    ROS_INFO("Waiting for offboard mode...");
                    last_request = ros::Time::now();
                    start_time = ros::Time::now();
                 }
            }

      	if (offb && armed)
      	{
      		chk1 = true; // Checks if z is less than 1.5
      		chk2 = (ros::Time::now() - start_time) < ros::Duration(7.0); //checks if 3 seconds have elapsed from start
      		
      		if (chk1 && chk2 )
      		{
      		    pos_sp.header.stamp = ros::Time::now();
		    pos_sp.header.frame_id = "fcu";
      		    pos_sp.pose.position.x = 0;
		    pos_sp.pose.position.y = 0;
		    pos_sp.pose.position.z = 1.4;
		    pos_setpoint_flag = true;
		}
			
		// chk3 = currentPose.pose.position.z >= 1.45;
		// chk4 = currentPose.pose.position.z < 1.55;
		chk5 = (ros::Time::now() - start_time) > ros::Duration(7.0);
		chk6 = (ros::Time::now() - start_time) < ros::Duration(11.0);
			
		if (chk6 && chk5 ) // if t>3 & 1.45<z<1.55
		{
			pos_sp.header.stamp = ros::Time::now();
			pos_sp.header.frame_id = "fcu";
			velo_sp.twist.linear.y = -0.75; //m/s
			pos_setpoint_flag = false;
		}
	
		chk7 = (ros::Time::now() - start_time) > ros::Duration(11.0);
		chk8 = (ros::Time::now() - start_time) < ros::Duration(13.0);

		if (chk7 && chk8)
		{
			pos_sp.header.stamp = ros::Time::now();
			pos_sp.header.frame_id = "fcu";
			velo_sp.twist.linear.y = 0; //m/s
			pos_setpoint_flag = false;				
		}

		chk9 = (ros::Time::now() - start_time) > ros::Duration(13.0);
		chk10 = (ros::Time::now() - start_time) < ros::Duration(17.0);

		if (chk9 )
		{
			pos_sp.header.stamp = ros::Time::now();
			pos_sp.header.frame_id = "fcu";
			pos_sp.pose.position.x = 0;
			pos_sp.pose.position.y = -3.3;
			pos_sp.pose.position.z = -0.25;
			pos_setpoint_flag = true;
			//velo_sp.twist.linear.y = -0.75; //m/s			
		}
/*(
		chk11 = (ros::Time::now() - start_time) > ros::Duration(17.0); 

		if (chk11)
		{
			pos_sp = home_pose;
			pos_setpoint_flag = true;

		}*/



	}

	if(pos_setpoint_flag)
	{
		mavros_pos_control_pub.publish(pos_sp);
	}
	else
	{
		local_vel_pub.publish(velo_sp);
	}
		
	ros::spinOnce();
	loop_rate.sleep();

    }

return 0;
}
