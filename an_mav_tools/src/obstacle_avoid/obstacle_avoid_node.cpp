/**
 * @file obstacle_avoid_node.cpp
 * @brief Obstacle avoidance example, written with mavros version 0.14.2, px4
 * flight stack, ros_control
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <control_toolbox/pid.h>

#include "an_mav_tools/SetpointManager.h"
#include "an_mav_tools/SetpointManagerVelocity.h"

geometry_msgs::PoseStamped local_pos;
geometry_msgs::Twist cmd_vel;

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos = *msg;
}

void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg){
    cmd_vel = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoid_node");
    ros::NodeHandle nh("~");

    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, &local_pos_cb);
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>
            ("/cmd_vel", 10, &cmd_vel_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    SetpointManager* sp_man = new SetpointManagerVelocity(nh);
    sp_man->Set(0.0, 0.0, 1.0);
    sp_man->Start();

    ros::Duration(2).sleep();

    //PID stuff
    control_toolbox::Pid pid_linvel_z;
    pid_linvel_z.initPid(0.38, 0.06, 0.12, 0.1, -0.1, nh);

    //main loop setup
    ros::Rate rate(20);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    bool offboard_enabled = false;
    bool armed = false;
    ros::Time last_time = ros::Time::now();

    while(ros::ok()){
        if(!offboard_enabled){
            // The mode switch should be accepted once a few setpoints are sent
            if(set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
                offboard_enabled = true;
            }
        } else {
            if(!armed){
                if(arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    armed = true;
                }
            }
        }

        double lin_vel_z = pid_linvel_z.computeCommand(
                    1.0 - local_pos.pose.position.z, ros::Time::now()-last_time);
        sp_man->Set(cmd_vel.linear.x, cmd_vel.linear.y, lin_vel_z);
        //ROS_INFO("Sending velocity: %G", lin_vel_z);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
