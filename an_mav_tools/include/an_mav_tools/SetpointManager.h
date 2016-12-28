#ifndef SETPOINT_MANAGER_H
#define SETPOINT_MANAGER_H

#include <ros/ros.h>
#include <thread>
#include <mutex>

class SetpointManager {
public:
    SetpointManager(ros::NodeHandle nh);

    void Start();
    void Stop();
    void Set(double x, double y, double z);

    ~SetpointManager();

protected:
    std::mutex settings_mutex_;

    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
    bool work_done_ = false;

    ros::Rate rate_;
    ros::NodeHandle nh_;
    std::thread* publish_thread_;

    virtual void Publish() = 0;
};

#endif //SETPOINT_MANAGER_H
