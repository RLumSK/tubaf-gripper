#ifndef OBJECTSEARCH_H
#define OBJECTSEARCH_H

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit_msgs/CollisionObject.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/package.h>

//OpenCV
#include "opencv2/surface_matching/ppf_helpers.hpp"

#include <tbf_gripper_tools/helperfunctions.h>

/** Abstract base class of the Object Search in ROS, it takes care of the publishing and gives a construct for the subscribing
 * @brief The ObjectSearch class abstract object search base class
 */
class ObjectSearch
{


protected:
  // ROS
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher pose_pub, model_pub, model_pcl_pub, scene_pcl_pub,  marker_pub, marker_array_pub;
  tf::TransformListener tf_listener;
  sensor_msgs::PointCloud2Ptr msg;

  // data and object
  moveit_msgs::CollisionObject ros_model_msg;
  visualization_msgs::Marker ros_marker_msg, dummy_marker;
  visualization_msgs::MarkerArray ros_marker_array_msg;

  std::string pcl_topic;

  /** Subscribe to a PointCloud topic by connecting the pcl_callback
   * @brief subscribe start pcl_callback
   */
  virtual void subscribe() = 0;

  /** Shutdown the PointCloud subscribtion and therefore stop the pcl_callback invocation
   * @brief unsubscribe stop pcl_callback
   */
  virtual void unsubscribe() = 0;

public:
  /* Standart constructor
   * @brief ObjectSearch
   * @param handle ROS node handle
   * @param model_name path to the model to be tracked
   * @param pcl_topic topic of the point cloud representing the scene
   */
  ObjectSearch(ros::NodeHandle& handle,const std::string& model_name,const std::string& pcl_topic);

  /** Bind this callback of the Pointcloud Subscriber with the current scene using the subscribe/unsubscribe methods
   * @brief pcl_callback current scene as point cloud
   * @param pcl_msg sensor_msgs::PointCloud2
   */
  virtual void pcl_callback(const sensor_msgs::PointCloud2& pcl_msg)=0;

  /** Publish the calculated object pose in ROS
   * @brief publish_pose pass to ROS
   * @param pcl_msg point cloud message used
   * @param pose estimated object pose
   */
  void publish_pose(const sensor_msgs::PointCloud2& pcl_msg, const geometry_msgs::Pose& pose, bool invert=false);
  void publish_pose(const sensor_msgs::PointCloud2& pcl_msg, Eigen::Matrix4f& transformation, bool invert=false);
  void publish_pose(const sensor_msgs::PointCloud2& pcl_msg, double* affine_transformation, bool invert=false);
 // void publish_pose(const sensor_msgs::PointCloud2& pcl_msg, double* position, double* quaternion, bool invert=false);


};

#endif // OBJECTSEARCH_H
