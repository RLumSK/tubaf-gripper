#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include "opencv2/core/utility.hpp"
#include "opencv2/surface_matching.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <typeinfo>
#include <sensor_msgs/PointCloud2.h>

#include "shape_msgs/Mesh.h"
#include <geometric_shapes/shape_operations.h>

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

#include "a.out.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;
using namespace ppf_match_3d;


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

/** Remove all row entries with a nan from the matrix
 * @brief removeNaN delete rows with nan
 * @param points reference to a point cloud matrix
 * @return a matrix without the nan rows
 */
static cv::Mat removeNaN(cv::Mat& points);

/** Replace all NaN entries in this matrix with a given number, therefore a mask is build and every refering entrie is substituted by the scalar
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
static void debug_print(const cv::Mat&, const cv::Mat&);

/** Generate a debug print of a OpenCV matrix and some parameter (rows, channels, ...)
 * @brief debug_print debug print of two matrixes
 */
static void debug_print(const cv::Mat&);

//static int computeNormalsCV(const Mat&, Mat&, const int, const bool, const Vec3d&);

/** Import the model from a given path and convert it to a ROS shape message
 * @brief import_model Import CAD model and convert to ROS message
 * @param path Path to the CAD file
 * @return ROS message with the model
 */
static shape_msgs::Mesh import_model(const std::string& path);

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

/** Convert a transformation stated in affine coordinates into a ROS pose
 * @brief eigenMatrix4f_to_pose Eigen Matrix Transformation to ROS pose
 * @param mat affine transformation in a 4x4 matrix
 * @param pose output of the ROS pose
 */
static void eigenMatrix4f_to_pose(const Eigen::Matrix4f& mat, geometry_msgs::Pose& pose);

/** Convert a transformation stated in affine coordinates into a ROS pose
 * @brief double16_to_pose double[16] to ROS pose
 * @param dbl_array array with 16 doubles refering to 4x4 matrix of an affine transformation
 * @param pose output of the ROS pose
 */
static void double16_to_pose(const double* dbl_array, geometry_msgs::Pose& pose);
};

#endif // HELPERFUNCTIONS_H
