/*
 * grid_map_pcl_loader_node.cpp
 *
 *  Created on: Aug 26, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */
#include <stdio.h>
#include <ros/ros.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>
using namespace boost::system;
namespace fs = boost::filesystem;

using namespace ros;
namespace gm = ::grid_map::grid_map_pcl;
using namespace grid_map;

int main(int argc, char** argv) {
  printf("======================================= This is Printf part =========================================\n");
  printf("%d\n\n", argc);
  for (int i = 0; i < argc; i++){
    printf("%s\n", argv[i]);
  }
  printf("======================================= Done Printf part =========================================\n");

  ros::init(argc, argv, "grid_map_pcls_loader_node");
  ros::NodeHandle nh("~");
  gm::setVerbosityLevelToDebugIfFlagSet(nh);

  // generate publisher
  ros::Publisher gridMapPub;
  gridMapPub = nh.advertise<grid_map_msgs::GridMap>("grid_map_from_raw_pointcloud", 1, true);

  // load pcl and parameter file
  grid_map::GridMapPclLoader gridMapPclLoader;
  gridMapPclLoader.loadParameters(gm::getParameterPath(nh));
  
  std::string pathToCloud;
  grid_map::GridMap gridMap;
  cv::Mat originalImage;
  std::string destination_path = "/home/junghwanyim/";
  std::string sorce_path = "/home/junghwanyim/catkin_ws/src/grid_map/grid_map_pcl/data/";
  // pathToCloud = sorce_path+"0000000103.pcd";
  for (const auto & entry : fs::directory_iterator(sorce_path)){
    std::cout << entry.path() << std::endl;
    pathToCloud = entry.path().c_str();
    gridMapPclLoader.loadCloudFromPcdFile(pathToCloud);

    // read pcl and process 
    gm::processPointcloud(&gridMapPclLoader, nh);
    gridMap = gridMapPclLoader.getGridMap();
    gridMap.setFrameId(gm::getMapFrame(nh));

    // save gridmap as rosbag file
    gm::saveGridMap(gridMap, nh, gm::getMapRosbagTopic(nh));

    // convert to CV image.    
    grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridMap, "elevation", CV_16UC1, 0.0, 100.0, originalImage);

    // save image
    std::string token = pathToCloud.substr(pathToCloud.find_last_of("/\\") + 1).c_str();
    
    imwrite(destination_path+token+".jpg", originalImage);
  }


  // publish grid map
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(gridMap, msg);
  gridMapPub.publish(msg);

  // run
  ros::spin();
  return EXIT_SUCCESS;
}
