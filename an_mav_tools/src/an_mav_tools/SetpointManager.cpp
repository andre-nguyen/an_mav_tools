#include <ros/ros.h>
#include <thread>
#include <mutex>

#include "an_mav_tools/SetpointManager.h"

SetpointManager::SetpointManager(ros::NodeHandle nh) : nh_(nh), rate_(10){

}

void SetpointManager::Start() {
    publish_thread_ = new std::thread(&SetpointManager::Publish, this);
}

void SetpointManager::Stop() {
    publish_thread_->join();
}

void SetpointManager::Set(double x, double y, double z) {
    //std::lock_guard<std::mutex> lock(settings_mutex_);
    x_ = x;
    y_ = y;
    z_ = z;
}

SetpointManager::~SetpointManager() {
    this->Stop();
}
