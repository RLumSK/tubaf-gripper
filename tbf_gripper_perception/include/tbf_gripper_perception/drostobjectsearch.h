#ifndef DROSTOBJECTSEARCH_H
#define DROSTOBJECTSEARCH_H

#include <limits.h>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/CollisionObject.h>
#include <object_recognition_msgs/ObjectType.h>
#include <geometry_msgs/Pose.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/features/normal_3d.h>

// OpenCV
#include "opencv2/core/utility.hpp"
#include "opencv2/surface_matching.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"

#include <tbf_gripper_tools/helperfunctions.h>

class DrostObjectSearch
{
private:
  // ROS
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher pose_pub, model_pub;

  sensor_msgs::PointCloud2Ptr msg;

  // surface_matching
  // parameter
  float sm_relSampleStep, sm_relDistanceStep, sm_icp_tolerance ,sm_icp_rejectionScale;
  int sm_numAngles, sm_icp_iterations, sm_icp_numLevels;

  // data and object
  cv::Mat pc_model;
  moveit_msgs::CollisionObject ros_model_msg;
  cv::ppf_match_3d::PPF3DDetector* detector;



public:
  DrostObjectSearch(ros::NodeHandle& handle,const std::string& model_path,const std::string& pcl_topic);
  ~DrostObjectSearch();

  /** Callback of the Pointcloud Subscriber with the current scene
   * @brief pcl_callback current scene as point cloud
   * @param pcl_msg sensor_msgs::PointCloud2
   */
  void pcl_callback(const sensor_msgs::PointCloud2& pcl_msg);

  /** Publishes the loaded model as PointCloud2 message to check the sampling of the given CAD model
   * @brief publish_model publish model as PointCloud2
   */
  void publish_model();

  /** Convert a given point cloud message to a scene representation by computing the normals for each point
   * @brief pointcloud_to_cvMat convert PointCloud2 to cv::Mat, interpret as scene
   * @param point_cloud sensor_msgs::PointCloud2
   * @param scene point cloud with normals as OpenCV matrix
   * @return true if succeddful
   */
  static bool pointcloud_to_cvMat(const sensor_msgs::PointCloud2& point_cloud, cv::Mat& scene);

  /** Convert a point cloud saved as Nx3 or Nx6 cv::Mat to a ROS message
   * @brief cvMat_to_pointcloud Convert a cv:Mat to PointCloud2
   * @param pcl cv::Mat point cloud (only first three columns/channels are used)
   * @param msg PointCloud2 message
   */
  static void cvMat_to_pointcloud(const cv::Mat& pcl, sensor_msgs::PointCloud2& msg);


};

#endif // DROSTOBJECTSEARCH_H
