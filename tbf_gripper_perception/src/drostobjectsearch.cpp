#include "../include/drostobjectsearch.h"

using namespace std;

DrostObjectSearch::DrostObjectSearch(ros::NodeHandle& handle, const string& model_path, const  string& pcl_topic):
  detector(0.025, 0.05),
  pc_model(cv::ppf_match_3d::loadPLYSimple(model_path.c_str(), 1))
{
  ROS_INFO_STREAM("[DrostObjectSearch::DrostObjectSearch()] " << "Model: " << model_path << endl <<
                  "Columns: " << pc_model.cols << endl <<
                  "MatSize: "<< *this->pc_model.size);

  HelperFunctions::debug_print(this->pc_model);

  // Surface Matching
  // Now train the model
  ROS_INFO_STREAM("[DrostObjectSearch::DrostObjectSearch()] " << "Training...");
  int64 tick1 = cv::getTickCount();
  this->pc_model.convertTo(this->pc_model, CV_32FC1);
  this->detector.trainModel(this->pc_model);
  int64 tick2 = cv::getTickCount();
  ROS_INFO_STREAM("[DrostObjectSearch::DrostObjectSearch()] " << "Training complete in "
       << (double)(tick2-tick1)/ cv::getTickFrequency()
       << " sec");

  // ROS
  this->nh = handle;
  this->pose_pub = this->nh.advertise<geometry_msgs::PoseStamped>("obj_pose", 1);
  this->pcl_sub = this->nh.subscribe(pcl_topic, 1, &DrostObjectSearch::pcl_callback, this);
}

void DrostObjectSearch::pcl_callback(const sensor_msgs::PointCloud2& pcl_msg)
{
  this->pcl_sub.shutdown();
  ROS_DEBUG_STREAM("[DrostObjectSearch::pcl_callback()] " << "It's a point cloud");

  // Convert the scene
  cv::Mat pcTest;
  if(!HelperFunctions::wrapper_pcl_normals_computation(pcl_msg, pcTest)){
    // Error during conversion
    ROS_WARN_STREAM("[DrostObjectSearch::pcl_callback()] " << "Convertion from the point cloud to scene failed [PCL]" );
    if(!DrostObjectSearch::pointcloud_as_scene(pcl_msg, pcTest)){
        // Error during conversion
        ROS_WARN_STREAM("[DrostObjectSearch::pcl_callback()] " << "Convertion from the point cloud to scene failed [OpenCV]" );
        //Restart subscriber
        this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &DrostObjectSearch::pcl_callback, this);
        return;
    }
  }
  ROS_DEBUG_STREAM("[DrostObjectSearch::pcl_callback()] " << "Converted point cloud to scene" );
  pcTest = HelperFunctions::removeNaN(pcTest); //- takes very long
  //HelperFunctions::replaceNaN(pcTest, FLT_MIN);

  // Match the model to the scene and get the pose
  ROS_INFO_STREAM("[DrostObjectSearch::pcl_callback()] " << "Starting matching..." );
  vector<cv::ppf_match_3d::Pose3DPtr> results;
  int64 tick1 = cv::getTickCount();
  this->detector.match(pcTest, results, 1.0/40.0, 0.05);
  int64 tick2 = cv::getTickCount();

  // Some informations about the matching
  ROS_INFO_STREAM("[DrostObjectSearch::pcl_callback()] " << "PPF Elapsed Time " <<
       (tick2-tick1)/cv::getTickFrequency() << " sec with " << results.size() << " results");
  cv::ppf_match_3d::Pose3DPtr pose_max_votes;
  unsigned int n_votes = 0;
  for(int i = 0; i< results.size(); i++){
    results[i]->printPose();
    if(n_votes < results[i]->numVotes){
      n_votes = results[i]->numVotes;
      pose_max_votes = results[i];
    }
  }

  // publish Pose with the most votes
  geometry_msgs::Pose retPose =  HelperFunctions::toROSPose(pose_max_votes);
  geometry_msgs::PoseStamped pubPose;
  pubPose.pose = retPose;
  pubPose.header = pcl_msg.header;
  this->pose_pub.publish(pubPose);

  //Restart subscriber
  this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &DrostObjectSearch::pcl_callback, this);

  // from: https://github.com/opencv/opencv_contrib/issues/464
  // You might as well simply not use ICP for this if the pose output from the detector is sufficient.
  // Another option is to use point to point ICP, since your surface normals are not describing much
  // (they are similar all over the place).

  // Create an instance of ICP
//  cv::ppf_match_3d::ICP icp(100, 0.005f, 2.5f, 8);
//  int64 t1 = cv::getTickCount();

//  // Register for all selected poses
//  ROS_INFO_STREAM("[DrostObjectSearch::pcl_callback()] " << "Performing ICP on " << N << " poses...");
//  HelperFunctions::debug_print(this->pc_model);
//  HelperFunctions::debug_print(pcTest);
  // icp.registerModelToScene(this->pc_model, pcTest, resultsSub); // ERROR

//  int64 t2 = cv::getTickCount();

//  ROS_INFO_STREAM("[DrostObjectSearch::pcl_callback()] " << "ICP Elapsed Time " <<
//       (t2-t1)/cv::getTickFrequency() << " sec" << endl);

}

bool DrostObjectSearch::pointcloud_as_scene(const sensor_msgs::PointCloud2 &point_cloud, cv::Mat& pointsAndNormals)
{
  ROS_INFO_STREAM("DrostObjectSearch::pointcloud_as_scene(): " << "Start" << endl);
  int datatype = HelperFunctions::ros_pcl_to_opencv_datatype(point_cloud.fields[0].datatype);
//  ROS_INFO_STREAM("DrostObjectSearch::pointcloud_as_scene(): " << "point_cloud.height=" << point_cloud.height << " point_cloud.width="<< point_cloud.width  << endl);
//  cv::Mat points = cv::Mat((point_cloud.height* point_cloud.width)/3, 3, // 3 columns with 1 channel -> [x, y, z; x, y, z; ...]
//                               CV_32FC1, (void*) &point_cloud.data[0] // with C++ 11:   static_cast<void*>(point_cloud.data())
//                               );
  // unstructured point cloud
  cv::Mat points = cv::Mat(point_cloud.height* point_cloud.width, 1, // 1 column with 3 channel -> [x, y, z; x, y, z; ...]
                               CV_32FC3, (void*) &point_cloud.data[0] // with C++ 11:   static_cast<void*>(point_cloud.data())
                               );
  points = points.reshape(1, point_cloud.height* point_cloud.width);
  //points = HelperFunctions::removeNaN(points);
  // https://github.com/opencv/opencv_contrib/blob/master/modules/surface_matching/samples/ppf_normal_computation.cpp
  pointsAndNormals = cv::Mat(point_cloud.height* point_cloud.width, 6,
                                     CV_32FC1, cv::Scalar(0));
//  cv::Mat points;
//  points = cv::Mat(point_cloud.height, point_cloud.width,  datatype, a_copy.data.data());
//  points = points.reshape(1, (point_cloud.height * point_cloud.width)/3);
  int num_neighbors = 1;
  bool b_flip_viewpoint = false;
  const double viewpoint[3] = {0,0,0};
//  HelperFunctions::debug_print(points, pointsAndNormals);
  //  //ERROR HERE ?
  int retValue = cv::ppf_match_3d::computeNormalsPC3d(points, pointsAndNormals, num_neighbors, b_flip_viewpoint, viewpoint);
  if(retValue){
    ROS_ERROR_STREAM("[DrostObjectSearch::pointcloud_as_scene] Error during cv::ppf_match_3d::computeNormalsPC3d(...): " << retValue);
    HelperFunctions::debug_print(points, pointsAndNormals);
    return false;
  }
  else
    ROS_INFO_STREAM("DrostObjectSearch::pointcloud_as_scene(): " << "Computed Normals" << endl);

  HelperFunctions::debug_print(points, pointsAndNormals);
}

