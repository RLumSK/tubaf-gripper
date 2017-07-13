#ifndef DROSTOBJECTSEARCH_H
#define DROSTOBJECTSEARCH_H

#include <limits.h>

#include <tbf_gripper_perception/objectsearch.h>

// ROS
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <geometric_shapes/shape_operations.h>
#include <object_recognition_msgs/ObjectType.h>
#include <geometry_msgs/Pose.h>
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

class DrostObjectSearch: public ObjectSearch
{
private:

  // surface_matching
  // parameter
  double sm_relSampleStep, sm_relDistanceStep, sm_relSceneSampleStep, sm_relSceneDistance, sm_icp_tolerance ,sm_icp_rejectionScale;
  int sm_numAngles, sm_icp_iterations, sm_icp_numLevels;
  dynamic_reconfigure::Server<tbf_gripper_perception::SurfaceMatchingConfig> server;
  dynamic_reconfigure::Server<tbf_gripper_perception::SurfaceMatchingConfig>::CallbackType f;

  DebugDetector* detector;

  cv::Mat pc_model;

protected:
  void subscribe();
  void unsubscribe();
public:
  DrostObjectSearch(ros::NodeHandle& handle,const std::string& model_name,const std::string& pcl_topic);
  ~DrostObjectSearch();

  /** Callback of the Pointcloud Subscriber with the current scene
   * @brief pcl_callback current scene as point cloud
   * @param pcl_msg sensor_msgs::PointCloud2
   */
  void pcl_callback(const sensor_msgs::PointCloud2& pcl_msg) override;

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
