#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

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
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

class HelperFunctions
{
private:

/** Compute the normals in a point cloud for every point using the PCL as presented here: http://pointclouds.org/documentation/tutorials/normal_estimation.php
 * @brief computeNormalsPCL calculate normals for the point cloud
 * @param cloud point cloud
 * @param normals normels of the point cloud
 */
static void computeNormalsPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

/** Compute the normals in a point cloud for every point using the PCL as presented here: http://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php#normal-estimation-using-integral-images
 * @brief computeNormalsPCL calculate normals for the point cloud
 * @param cloud point cloud
 * @param normals normels of the point cloud
 */
static void computeOrganisedNormalsPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);


public:
/** Convert the ROS datatype of a pointcloud to the OpenCV notation by a simple lookup, assuming three channels
 * @brief ros_pcl_to_opencv_datatype convert ROS to OpenCV type
 * @param ros_type int
 * @return opencv type
 */
static int ros_pcl_to_opencv_datatype(int ros_type);

/**Remove all row entries with a nan from the matrix
 * @brief removeNaN delete rows with nan
 * @param points reference to a point cloud matrix
 * @return a matrix without the nan rows
 */
static cv::Mat removeNaN(cv::Mat& points);

/**Replace all NaN entries in this matrix with a given number, therefore a mask is build and every refering entrie is substituted by the scalar
 * @brief replaceNan replace NaN in matrix with scalar
 * @param points matrix containing NaN's
 * @param scalar substitute value
 */
static void replaceNaN(cv::Mat& points, float scalar);

/** Wrap the PCL Normal computation functionalitie to access it directly from the ROS/OpenCV ecosystem
 * @brief wrapper_pcl_normals_computation
 * @param point_cloud point_cloud msg from the subscriber
 * @param scene OpenCV matrix with points and normals
 * @return true if successful
 */
static bool wrapper_pcl_normals_computation(const sensor_msgs::PointCloud2& point_cloud, cv::Mat& scene);

/** Convert a pose from OpenCV to ROS
 * @brief toROSPose conversion to ROS
 * @return pose in ROS object
 */
static geometry_msgs::Pose toROSPose(cv::ppf_match_3d::Pose3DPtr cv_pose);

/** Generate a debug print of a OpenCV matrix and some parameter (rows, channels, ...)
 * @brief debug_print debug print of two matrixes
 */
static void debug_print(cv::Mat&, cv::Mat&);

/** Generate a debug print of a OpenCV matrix and some parameter (rows, channels, ...)
 * @brief debug_print debug print of two matrixes
 */
static void debug_print(cv::Mat&);
};

#endif // HELPERFUNCTIONS_H
