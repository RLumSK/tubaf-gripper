#ifndef ICPOBJECTSEARCH_H
#define ICPOBJECTSEARCH_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>

// Parameter
#include <dynamic_reconfigure/server.h>
#include <tbf_gripper_perception/IcpMatchingConfig.h>

// OpenCV
#include "opencv2/core/utility.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"

#include <tbf_gripper_tools/helperfunctions.h>

class IcpObjectSearch
{
private:
  // ROS
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher pose_pub;

  cv::Mat model, scene;
  sensor_msgs::PointCloud2 model_msg;

  // parameter
  double cv_icp_tolerance ,cv_icp_rejectionScale;
  int cv_icp_iterations, cv_icp_numLevels;
  dynamic_reconfigure::Server<tbf_gripper_perception::IcpMatchingConfig> server;
  dynamic_reconfigure::Server<tbf_gripper_perception::IcpMatchingConfig>::CallbackType f;


  cv::ppf_match_3d::ICP cv_icp;

public:

  IcpObjectSearch(ros::NodeHandle& handle,const std::string& model_name,const std::string& pcl_topic);

  /** Callback of the Pointcloud Subscriber with the current scene
   * @brief pcl_callback current scene as point cloud
   * @param pcl_msg sensor_msgs::PointCloud2
   */
  void pcl_callback(const sensor_msgs::PointCloud2& pcl_msg);

  /** Publishes the loaded model as PointCloud2 message to check the sampling of the given CAD model
   * @brief publish_model publish model as PointCloud2
   */
  void publish_model();


  /** Pass parameter changes to the node; see dynamic_reconfigure tutorials
   * @brief config_callback Change parameters
   * @param config new cofiguration
   * @param level level, which is the result of ORing together all of level values of the parameters that have changed
   */
  void config_callback(tbf_gripper_perception::IcpMatchingConfig &config, uint32_t level);

};

#endif // ICPOBJECTSEARCH_H
