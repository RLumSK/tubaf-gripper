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
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>

// Parameter
#include <dynamic_reconfigure/server.h>
#include <tbf_gripper_perception/SurfaceMatchingConfig.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/features/normal_3d.h>

// OpenCV
#include "opencv2/core/utility.hpp"
#include "opencv2/surface_matching.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"

#include <tbf_gripper_tools/helperfunctions.h>

class DebugDetector: public cv::ppf_match_3d::PPF3DDetector{
public:
  DebugDetector (const double relativeSamplingStep, const double relativeDistanceStep=0.05, const double numAngles=30)
    :cv::ppf_match_3d::PPF3DDetector(relativeSamplingStep, relativeDistanceStep, numAngles)
  {

  }

  cv::Mat getSampledPC(){
    return this->sampled_pc;
  }
};

class DrostObjectSearch
{
private:
  // ROS
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher pose_pub, model_pub, model_pcl_pub, scene_pcl_pub,  marker_pub, marker_array_pub;
  tf::TransformListener tf_listener;
  sensor_msgs::PointCloud2Ptr msg;

  // surface_matching
  // parameter
  double sm_relSampleStep, sm_relDistanceStep, sm_relSceneSampleStep, sm_relSceneDistance, sm_icp_tolerance ,sm_icp_rejectionScale;
  int sm_numAngles, sm_icp_iterations, sm_icp_numLevels;
  dynamic_reconfigure::Server<tbf_gripper_perception::SurfaceMatchingConfig> server;
  dynamic_reconfigure::Server<tbf_gripper_perception::SurfaceMatchingConfig>::CallbackType f;

  // data and object
  cv::Mat pc_model;
  moveit_msgs::CollisionObject ros_model_msg;
  visualization_msgs::Marker ros_marker_msg;
  visualization_msgs::MarkerArray ros_marker_array_msg;
  visualization_msgs::Marker dummy_marker;
  DebugDetector* detector;



public:
  DrostObjectSearch(ros::NodeHandle& handle,const std::string& model_name,const std::string& pcl_topic);
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

  /** Train the detector with the provided model and a set of parameters
   * @brief trainDetector Train the detector
   */
  void trainDetector();

  /** Pass parameter changes to the node; see dynamic_reconfigure tutorials
   * @brief config_callback Change parameters
   * @param config new cofiguration
   * @param level level, which is the result of ORing together all of level values of the parameters that have changed
   */
  void config_callback(tbf_gripper_perception::SurfaceMatchingConfig &config, uint32_t level);

};

#endif // DROSTOBJECTSEARCH_H
