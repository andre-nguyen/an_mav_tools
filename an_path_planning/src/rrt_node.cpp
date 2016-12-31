#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

bool gmap_init = false;
grid_map::GridMap gmap;

const double gresolution = 0.1;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(!gmap_init) {
    grid_map::GridMapRosConverter::initializeFromImage(*msg, gresolution, gmap);
    gmap_init = true;
  }
  grid_map::GridMapRosConverter::addLayerFromImage(*msg, "obstacles", gmap, 0.0, 1.0);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrt_node");
  ros::NodeHandle nh("~");
  ros::Subscriber image_sub = nh.subscribe("image", 1, imageCallback);
  ros::Publisher gridmap_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  ros::Rate r(30);
  while(ros::ok() && !gmap_init){
    ros::spinOnce();
    r.sleep();
  }

  // do something

  return 0;
}
