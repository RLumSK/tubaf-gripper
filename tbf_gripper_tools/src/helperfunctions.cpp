#include <tbf_gripper_tools/helperfunctions.h>

using namespace std;

void HelperFunctions::debug_print(const cv::Mat &points,const cv::Mat &pointsAndNormals){

  ROS_INFO_STREAM("HelperFunctions::debug_print():" << std::endl <<
                  "points <type>" << typeid(points.at<float>(0,0)).name() <<
                  "points.rows=" << points.rows << std::endl <<
                  "points.cols=" << points.cols << std::endl <<
                  "points.channels=" << points.channels() << std::endl <<
                  "points.step=" << points.step << std::endl <<
                  "pointsAndNormals.rows=" << pointsAndNormals.rows << std::endl <<
                  "pointsAndNormals.cols=" << pointsAndNormals.cols << std::endl <<
                  "pointsAndNormals.channels=" << pointsAndNormals.channels() << std::endl <<
                  "pointsAndNormals.step=" << pointsAndNormals.step);
  for(int i = 0; i < points.rows; i+=points.rows/5)
            cout << "points["<< i << "] \t" << points.at<float>(i,0) << " " << points.at<float>(i,1) << " " << points.at<float>(i,2) <<std::endl <<
                    "pntNor["<< i << "] \t" << pointsAndNormals.at<float>(i,0) << " " << pointsAndNormals.at<float>(i,1) << " " << pointsAndNormals.at<float>(i,2) << " \t" <<
                      pointsAndNormals.at<float>(i,3) << " " << pointsAndNormals.at<float>(i,4) << " " << pointsAndNormals.at<float>(i,5) <<
                    std::endl;

}

void HelperFunctions::debug_print(const cv::Mat &points){
  ROS_INFO_STREAM("HelperFunctions::debug_print():" << std::endl <<
                  "points.rows=" << points.rows << std::endl <<
                  "points.cols=" << points.cols << std::endl <<
                  "points.channels=" << points.channels() << std::endl <<
                  "points.step=" << points.step << std::endl);
  for(int i = 0; i < points.rows; i+=points.rows/5)
    if(points.step < 24)
            cout << "points["<< i << "] \t" << points.at<float>(i,0) << " " << points.at<float>(i,1) << " " << points.at<float>(i,2) << std::endl;
    else
            cout << "points["<< i << "] \t" << points.at<float>(i,0) << " " << points.at<float>(i,1) << " " << points.at<float>(i,2) << " \t" <<
                    points.at<float>(i,3) << " " << points.at<float>(i,4) << " " << points.at<float>(i,5) << std::endl;

}

int HelperFunctions::ros_pcl_to_opencv_datatype(int ros_type)
{
  int retValue = -1;
  switch (ros_type) {
  case 1: // INT8
    retValue = CV_8SC1;
    break;
  case 2: // UINT8
    retValue = CV_8UC1;
    break;
  case 3: // INT16
    retValue = CV_16SC1;
    break;
  case 4: // UINT16
    retValue = CV_16UC1;
    break;
  case 5: // INT32
    retValue = CV_32SC1;
    break;
  case 6: // UINT32
//    retValue = CV_32UC3; NOT KNOWN
    ROS_ERROR_STREAM("[DrostObjectSearch::ros_pcl_to_opencv_datatype()] Illegal ROS Type UINT32 - not known in opencv");
    break;
  case 7: // FLOAT32
    retValue = CV_32FC1;
    break;
  case 8: // FLOAT64
    retValue = CV_64FC1;
    break;
  default:
    break;
  }
  return retValue;
}

cv::Mat HelperFunctions::removeNaN(cv::Mat &points)
{
  cv::Mat result;
  bool first_entry = true;
  for(int i=0; i<points.rows; i++){
    const float* Mi = points.ptr<float>(i);
    if(Mi[0] != Mi[0] || Mi[1] != Mi[1] || Mi[2] != Mi[2]){ //is not NaN
      continue;
    }
    else{
      cv::Mat row = points.row(i);
      if(first_entry){
        result = cv::Mat(row);
        first_entry = false;
      }
      else{
        vconcat(result, row, result);
      }
    }
  }
  return result;
}

void HelperFunctions::replaceNaN(cv::Mat &points, float scalar)
{
  // see: http://answers.opencv.org/question/2221/create-a-mask-for-nan-cells/
  cv::Mat mask = cv::Mat(points != points);
  cv::Mat replacements(points.rows, points.cols, points.type(), scalar);
  replacements.copyTo(points, mask);
}

bool HelperFunctions::wrapper_pcl_normals_computation(const sensor_msgs::PointCloud2 &point_cloud, cv::Mat &scene)
{
  // Why by PointCloud::Ptr? https://github.com/hcrlab/wiki/blob/master/pcl/pcl_cpp_tips.md
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(point_cloud, *cloud_ptr);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  HelperFunctions::computeNormalsPCL(cloud_ptr, normals);
//  HelperFunctions::computeOrganisedNormalsPCL(cloud_ptr, normals);
  // Convert to OpenCV
  cv::Mat cv_points = cv::Mat(cloud_ptr->points.size(),1, // 1 column with 3 channel -> [x, y, z; x, y, z; ...]
                               CV_32FC3, (void*) &cloud_ptr->points[0] // with C++ 11:   static_cast<void*>(point_cloud.data())
                          );
  cv::Mat cv_normals = cv::Mat(normals->points.size(),1,
                               CV_32FC3, (void*) &normals->points[0]
                          );
  cv::hconcat(cv_points, cv_normals, scene);
  scene = scene.reshape(1,0);
  //HelperFunctions::debug_print(cv_points, scene);
  ROS_DEBUG_STREAM("HelperFunctions::wrapper_pcl_normals_computation(): Rows[points, normals, scene]:\t" << cv_points.rows << " " << cv_normals.rows << " "  << scene.rows << " "  );
  ROS_DEBUG_STREAM("HelperFunctions::wrapper_pcl_normals_computation(): Columns[points, normals, scene]:\t" << cv_points.cols << " " << cv_normals.cols << " "  << scene.cols << " "  );
  ROS_DEBUG_STREAM("HelperFunctions::wrapper_pcl_normals_computation(): Channels[points, normals, scene]:\t" << cv_points.channels() << " " << cv_normals.channels() << " "  << scene.channels() << " "  );
  return true;
}

geometry_msgs::Pose HelperFunctions::toROSPose(cv::ppf_match_3d::Pose3DPtr cv_pose)
{
  geometry_msgs::Pose retPose;

  retPose.position.x = cv_pose->t[0];
  retPose.position.y = cv_pose->t[1];
  retPose.position.z = cv_pose->t[2];

  retPose.orientation. x = cv_pose->q[0];
  retPose.orientation. y = cv_pose->q[1];
  retPose.orientation. z = cv_pose->q[2];
  retPose.orientation. w = cv_pose->q[3];

  return retPose;
}

void HelperFunctions::computeNormalsPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
    // see: http://pointclouds.org/documentation/tutorials/normal_estimation.php

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);
}

void HelperFunctions::computeOrganisedNormalsPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*cloud_normals);
}


shape_msgs::Mesh HelperFunctions::import_model(const string& path)
{
  // http://answers.ros.org/question/245995/adding-collision-object-in-moveit/
    string p = "file://"+path;
    ROS_INFO_STREAM("[HelperFunctions::import_model] Import from path: " << p);
    shapes::Mesh* m = shapes::createMeshFromResource(p);
    //m->scale(0.1);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);

    return boost::get<shape_msgs::Mesh>(mesh_msg);
}


bool HelperFunctions::pointcloud_to_cvMat(const sensor_msgs::PointCloud2 &point_cloud, cv::Mat& pointsAndNormals)
{
  ROS_DEBUG_STREAM("DrostObjectSearch::pointcloud_to_cvMat(): " << "Start" << endl);
  // unstructured point cloud
  cv::Mat points = cv::Mat(point_cloud.height* point_cloud.width, 3, // 3 columns with 1 channel -> [x, y, z; x, y, z; ...]
                               CV_32FC1, (void*) &point_cloud.data[0] // with C++ 11:   static_cast<void*>(point_cloud.data())
                               );
  //points = points.reshape(1, point_cloud.height* point_cloud.width);
  //points = HelperFunctions::removeNaN(points);
  // https://github.com/opencv/opencv_contrib/blob/master/modules/surface_matching/samples/ppf_normal_computation.cpp
  pointsAndNormals = cv::Mat(point_cloud.height* point_cloud.width, 6,CV_32FC1, cv::Scalar(0));
  int num_neighbors = 6;
  bool b_flip_viewpoint = false;
  const double viewpoint[3] = {0,0,0};
//  HelperFunctions::debug_print(points, pointsAndNormals);
  //  //ERROR HERE ?
  int retValue = cv::ppf_match_3d::computeNormalsPC3d(points, pointsAndNormals, num_neighbors, b_flip_viewpoint, viewpoint);
  //HelperFunctions::debug_print(points, pointsAndNormals);
  ROS_DEBUG_STREAM("DrostObjectSearch::pointcloud_to_cvMat(): " << "End" << endl);
  return retValue == 1;
}

void HelperFunctions::cvMat_to_pointcloud(const cv::Mat &cv_pcl, sensor_msgs::PointCloud2 &msg)
{
  //HelperFunctions::debug_print(cv_pcl);
  //reshape?
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (int y=0;y<cv_pcl.rows;y++)
  {
    pcl::PointXYZ point;
    point.x = cv_pcl.at<float>(y, 0);
    point.y = cv_pcl.at<float>(y, 1);
    point.z = cv_pcl.at<float>(y, 2);
    cloud->points.push_back(point);
  }
  pcl::toROSMsg(*cloud, msg);
}

void HelperFunctions::eigenMatrix4f_to_pose(const Eigen::Matrix4f &mat, geometry_msgs::Pose &pose)
{
  if(mat(3,3) != 1.0){
    ROS_WARN_STREAM("[HelperFunctions::eigenMatrix4f_to_pose] " << "mat(3,3) = " << mat(3,3) << " but should be 1");
  }
  pose.position.x = mat(0,3);
  pose.position.y = mat(1,3);
  pose.position.z = mat(2,3);
  Eigen::Quaternionf quat(mat.block<3,3>(0,0));
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
}

void HelperFunctions::double16_to_pose(const double *dbl_array, geometry_msgs::Pose &pose)
{
  if(dbl_array[15] != 1.0){
    ROS_WARN_STREAM("[HelperFunctions::double16_to_pose] " << "dbl_array[15] = " << dbl_array[15] << " but should be 1");
  }
  pose.position.x = static_cast<float>(dbl_array[3]);
  pose.position.y = static_cast<float>(dbl_array[7]);
  pose.position.z = static_cast<float>(dbl_array[11]);
  Eigen::Matrix3f mat;
  mat << static_cast<float>(dbl_array[0]) , static_cast<float>(dbl_array[1]), static_cast<float>(dbl_array[2]),
      static_cast<float>(dbl_array[4]) , static_cast<float>(dbl_array[5]), static_cast<float>(dbl_array[6]),
      static_cast<float>(dbl_array[8]) , static_cast<float>(dbl_array[9]), static_cast<float>(dbl_array[10]);
  Eigen::Quaternionf quat(mat);
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
}
