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

using namespace ros;
namespace gm = ::grid_map::grid_map_pcl;
using namespace grid_map;
// using namespace cv;

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
  const std::string pathToCloud = gm::getPcdFilePath(nh);
  gridMapPclLoader.loadParameters(gm::getParameterPath(nh));
  gridMapPclLoader.loadCloudFromPcdFile(pathToCloud);

  printf("pathToCloud %s\n", gm::getPcdFilePath(nh).c_str());
  printf("pathToParameter %s\n", gm::getParameterPath(nh).c_str());

  // read pcl and process 
  gm::processPointcloud(&gridMapPclLoader, nh);
  grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
  gridMap.setFrameId(gm::getMapFrame(nh));

  // save gridmap as rosbag file
  gm::saveGridMap(gridMap, nh, gm::getMapRosbagTopic(nh));

  // Convert to CV image.
  cv::Mat originalImage;
  grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridMap, "elevation", CV_16UC1, 0.0, 0.3, originalImage);

  // // Create OpenCV window.
  imwrite("/home/junghwanyim/test.jpg", originalImage);

  // publish grid map
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(gridMap, msg);
  gridMapPub.publish(msg);

  // run
  ros::spin();
  return EXIT_SUCCESS;
}
