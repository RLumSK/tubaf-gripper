#ifndef PCLICPOBJECTSEARCH_H
#define PCLICPOBJECTSEARCH_H

#include <tbf_gripper_perception/icpobjectsearch.h>

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

class PclIcpObjectSearch: public IcpObjectSearch
{
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_model, pcl_scene;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> pcl_icp;

protected:
  void subscribe() override;
  void unsubscribe() override;


public:

  PclIcpObjectSearch(ros::NodeHandle& handle,const std::string& model_name,const std::string& pcl_topic);

    /** Callback of the Pointcloud Subscriber with the current scene
   * @brief pcl_callback current scene as point cloud
   * @param pcl_msg sensor_msgs::PointCloud2
   */
  void pcl_callback(const sensor_msgs::PointCloud2& pcl_msg) override;

};
#endif // PCLICPOBJECTSEARCH_H
