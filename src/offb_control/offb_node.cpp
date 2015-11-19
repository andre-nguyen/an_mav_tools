/**
 * @file offb_node.cpp
 * @brief Example node for offboard control using ROS Jade, mavros 0.14.2 and
 * the px4 flight stack.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh("~");

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    int i = 100;
    while(ros::ok() && i > 0){
        local_pos_pub.publish(pose);
        i--;
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    bool offboard_enabled = false;
    bool armed = false;

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

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
