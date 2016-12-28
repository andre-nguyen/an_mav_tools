#ifndef SETPOINT_MANAGER_VELOCITY_H
#define SETPOINT_MANAGER_VELOCITY_H

#include <ros/ros.h>

#include "SetpointManager.h"

class SetpointManagerVelocity : public SetpointManager {
public:
    SetpointManagerVelocity(ros::NodeHandle nh);
private:
    ros::Publisher vel_sp_pub_;

    void Publish();
};

#endif //SETPOINT_MANAGER_VELOCITY_H
