#include <ros/ros.h>
#include <mutex>
#include <geometry_msgs/TwistStamped.h>

#include "an_mav_tools/SetpointManager.h"
#include "an_mav_tools/SetpointManagerVelocity.h"

SetpointManagerVelocity::SetpointManagerVelocity(ros::NodeHandle nh) :
    SetpointManager(nh){
    vel_sp_pub_ = nh_.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10);
}

void SetpointManagerVelocity::Publish() {
    while(ros::ok() && !work_done_) {
        //std::lock_guard<std::mutex> lock(settings_mutex_);
        geometry_msgs::TwistStamped vel_sp;
        vel_sp.twist.linear.x = x_;
        vel_sp.twist.linear.y = y_;
        vel_sp.twist.linear.z = z_;
        vel_sp.twist.angular.x = vel_sp.twist.angular.y = vel_sp.twist.angular.z = 0;
        vel_sp.header.frame_id = "base_footprint";
        vel_sp.header.stamp = ros::Time::now();

        vel_sp_pub_.publish(vel_sp);

        rate_.sleep();
    }
}
