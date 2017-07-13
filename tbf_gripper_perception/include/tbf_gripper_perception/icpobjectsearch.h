#ifndef ICPOBJECTSEARCH_H
#define ICPOBJECTSEARCH_H

#include <tbf_gripper_perception/objectsearch.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

// Parameter
#include <dynamic_reconfigure/server.h>
#include <tbf_gripper_perception/IcpMatchingConfig.h>

// OpenCV
#include "opencv2/core/utility.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"

// PCL
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>

#include <tbf_gripper_tools/helperfunctions.h>

class IcpObjectSearch: public ObjectSearch
{
private:

  cv::Mat cv_model, cv_scene;
  cv::ppf_match_3d::ICP cv_icp;

protected:
  // parameter
  double icp_tolerance ,icp_rejectionScale;
  int icp_iterations, icp_numLevels;
  dynamic_reconfigure::Server<tbf_gripper_perception::IcpMatchingConfig> server;
  dynamic_reconfigure::Server<tbf_gripper_perception::IcpMatchingConfig>::CallbackType f;


public:

  IcpObjectSearch(ros::NodeHandle& handle,const std::string& model_name,const std::string& pcl_topic);

    /** Callback of the Pointcloud Subscriber with the current scene
   * @brief pcl_callback current scene as point cloud
   * @param pcl_msg sensor_msgs::PointCloud2
   */
  virtual void pcl_callback(const sensor_msgs::PointCloud2& pcl_msg)=0;

//  /** Callback of the Pointcloud Subscriber with the current scene, using the OpenCV implementation to perform an ICP for pose estimation
//   * @brief pcl_callback current scene as point cloud
//   * @param pcl_msg sensor_msgs::PointCloud2
//   */
//  void cv_icp_callback(const sensor_msgs::PointCloud2ConstPtr &pcl_msg);

//  /** Callback of the Pointcloud Subscriber with the current scene, using the PCL implementation to perform an ICP for pose estimation
//   * @brief pcl_callback current scene as point cloud
//   * @param pcl_msg sensor_msgs::PointCloud2
//   */
//  void pcl_icp_callback(const sensor_msgs::PointCloud2ConstPtr &pcl_msg);


//  /** Publishes the loaded model as PointCloud2 message to check the sampling of the given CAD model
//   * @brief publish_model publish model as PointCloud2
//   */
//  void publish_model();


  /** Pass parameter changes to the node; see dynamic_reconfigure tutorials
   * @brief config_callback Change parameters
   * @param config new cofiguration
   * @param level level, which is the result of ORing together all of level values of the parameters that have changed
   */
  virtual void config_callback(tbf_gripper_perception::IcpMatchingConfig &config, uint32_t level);

};

#endif // ICPOBJECTSEARCH_H
