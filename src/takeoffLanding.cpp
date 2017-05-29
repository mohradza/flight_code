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
	ros::init(argc, argv, "px4_AVL_Node");
	
	ros::NodeHandle n;

	// ROS Publisher
	ros::Publisher mavros_pos_control_pub = n.advertise<geometry_msgs::PoseStamped>
			("mavros/setpoint_position/local",100);

	// ROS Subscriber 
	ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
			("mavros/state",100, state_callback);

	ros::Subscriber local_pos_sub = n.subscribe<geometry_msgs::PoseStamped>
			("mavros/local_position/local",100,pose_callback);

	// ROS Client/Services:
	ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
	ros::ServiceClient mavros_set_mode_client = n.serviceClient<mavros_msgs::SetMode>
			("/mavros/set_mode");
  	mavros_msgs::SetMode set_mode;
  	set_mode.request.custom_mode = "OFFBOARD";

  	bool offboard_commands_enabled = false;

  	ros::Rate loop_rate(100.0);

  	geometry_msgs::PoseStamped pos_sp; // object for set point
  	geometry_msgs::PoseStamped home_pose;
  	geometry_msgs::PoseStamped landing_pose;
  	geometry_msgs::PoseStamped takeoff_pose;
  	home_pose = currentPose;
  	landing_pose = currentPose;
  	landing_pose.pose.position.z = currentPose.pose.position.z - 0.15;


  	for(int i = 100; ros::ok() && i > 0; --i){
        mavros_pos_control_pub.publish(home_pose);
        ros::spinOnce();
        loop_rate.sleep();
    }


  	while (ros::ok())
  	{
  		
  		if (!offboard_commands_enabled)
  		{
  			if (mavros_set_mode_client.call(set_mode))
      		{
      			ROS_INFO("Set mode: OFFBOARD enabled!");
				offboard_commands_enabled = true;
      		}
      	}
      	else
      	{
        	ROS_INFO("Offboard mode still not enabled!");
      	}
      	

      	//if (offboard_commands_enabled)
      	{
      		if (currentPose.pose.position.z < 1.5)
      		{
      			pos_sp.header.stamp = ros::Time::now();
				pos_sp.header.frame_id = "fcu";
				pos_sp.pose.position.x = 0;
				pos_sp.pose.position.y = 0;
				pos_sp.pose.position.z = 1.5;
			} /*
			else if (currentPose.pose.position.z >= 1.4)
			{
				pos_sp.header.stamp = ros::Time::now();
				pos_sp.header.frame_id = "fcu";
				pos_sp.pose.position.x = 0;
				pos_sp.pose.position.y = 0;
				pos_sp.pose.position.z = 1.5;
			}*/

			else if (currentPose.pose.position.z >= 1.4)
			{
				pos_sp.header.stamp = ros::Time::now();
				pos_sp.header.frame_id = "fcu";
				pos_sp.pose.position.x = 0;
				pos_sp.pose.position.y = 0;
				pos_sp.pose.position.z = -.015;
			}

		}

		mavros_pos_control_pub.publish(pos_sp);
		ros::spinOnce();
		loop_rate.sleep();

    }

	return 0;
}