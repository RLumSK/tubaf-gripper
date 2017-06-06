#ifndef DROSTOBJECTSEARCH_H
#define DROSTOBJECTSEARCH_H

#include <limits.h>

#include "opencv2/core/utility.hpp"
#include "opencv2/surface_matching.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/features/normal_3d.h>

//OpenCV

#include "helperfunctions.h"

class DrostObjectSearch
{
private:
  // ROS
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher pose_pub;

  // surface_matching
  cv::Mat pc_model;
  cv::ppf_match_3d::PPF3DDetector detector;

public:
  DrostObjectSearch(ros::NodeHandle& handle,const std::string& model_path,const std::string& pcl_topic);

  /** Callback of the Pointcloud Subscriber with the current scene
   * @brief pcl_callback current scene as point cloud
   * @param pcl_msg sensor_msgs::PointCloud2
   */
  void pcl_callback(const sensor_msgs::PointCloud2& pcl_msg);

  /** Convert a given point cloud message to a scene representation by computing the normals for each point
   * @brief pointcloud_as_scene convert PointCloud2 to cv::Mat, interpret as scene
   * @param point_cloud sensor_msgs::PointCloud2
   * @param scene point cloud with normals as OpenCV matrix
   * @return true if succeddful
   */
  static bool pointcloud_as_scene(const sensor_msgs::PointCloud2& point_cloud, cv::Mat& scene);
};

#endif // DROSTOBJECTSEARCH_H
