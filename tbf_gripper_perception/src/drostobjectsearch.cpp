#include "tbf_gripper_perception/drostobjectsearch.h"

using namespace std;

DrostObjectSearch::DrostObjectSearch(ros::NodeHandle& handle, const string& model_name, const  string& pcl_topic):
  nh(handle),
  pcl_sub(nh.subscribe(pcl_topic, 1, &DrostObjectSearch::pcl_callback, this)),
  pose_pub(nh.advertise<geometry_msgs::PoseStamped>("/perception/obj_pose", 1)),
  model_pub(nh.advertise<moveit_msgs::CollisionObject>("/perception/model_pcl", 1)),
  marker_pub(nh.advertise<visualization_msgs::Marker>("/perception/object", 1)),
  msg(new sensor_msgs::PointCloud2),
  pc_model(cv::ppf_match_3d::loadPLYSimple((ros::package::getPath("tbf_gripper_perception") +"/meshes/" + model_name +".ply").c_str(), 1)),
  ros_model_msg(),
  ros_marker_msg()
{
  nh.param<double>("sm_relSampleStep", sm_relSampleStep, 0.5);
  nh.param<double>("sm_relDistanceStep", sm_relDistanceStep, 0.05);
  nh.param<double>("sm_relSceneSampleStep", sm_relSceneSampleStep, 0.2);
  nh.param<double>("sm_relSceneDistance", sm_relSceneDistance, 0.03);
  nh.param<double>("sm_icp_tolerance", sm_icp_tolerance, 0.05);
  nh.param<double>("sm_icp_rejectionScale", sm_icp_rejectionScale, 2.5);
  nh.param<int>("sm_numAngles", sm_numAngles, 30);
  nh.param<int>("sm_icp_iterations", sm_icp_iterations, 30);
  nh.param<int>("sm_icp_numLevels", sm_icp_numLevels, 30);
  ros_model_msg.id = "object";
  ros_model_msg.operation = moveit_msgs::CollisionObject::ADD;
  ros_model_msg.meshes.push_back(HelperFunctions::import_model(ros::package::getPath("tbf_gripper_perception") +"/meshes/" + model_name + ".stl"));
  ros_model_msg.mesh_poses.push_back(geometry_msgs::Pose());
  ros_marker_msg.ns = "object";
  ros_marker_msg.id = 0;
  ros_marker_msg.type = visualization_msgs::Marker::MESH_RESOURCE;
  ros_marker_msg.action = visualization_msgs::Marker::ADD;
  ros_marker_msg.scale.x = 1.0;
  ros_marker_msg.scale.y = 1.0;
  ros_marker_msg.scale.z = 1.0;
  ros_marker_msg.color.a = 1.0;
  ros_marker_msg.color.r = 0.0;
  ros_marker_msg.color.g = 0.0;
  ros_marker_msg.color.b = 1.0;
  ros_marker_msg.mesh_resource = "package://tbf_gripper_perception/meshes/"+ model_name +".stl";
//      "file://"+model_path+".stl"; // Can be any mesh type supported by rviz (binary .stl or Ogre .mesh in 1.0, with the addition of COLLADA (.dae) in 1.1). The format is the URI-form used by resource_retriever, including the package:// syntax.

  detector = new cv::ppf_match_3d::PPF3DDetector(sm_relSampleStep, sm_relDistanceStep, sm_numAngles);

  ROS_DEBUG_STREAM("[DrostObjectSearch::DrostObjectSearch()] " << "Model: " << model_name << endl <<
                  "Columns: " << pc_model.cols << endl <<
                  "MatSize: "<< *this->pc_model.size);

  // Surface Matching
  // Now train the model
  ROS_DEBUG_STREAM("[DrostObjectSearch::DrostObjectSearch()] " << "Training...");
  int64 tick1 = cv::getTickCount();
  //this->pc_model.convertTo(this->pc_model, CV_32FC1);
  this->detector->trainModel(this->pc_model);
  int64 tick2 = cv::getTickCount();
  ROS_INFO_STREAM("[DrostObjectSearch::DrostObjectSearch()] " << "Training complete in "
       << (double)(tick2-tick1)/ cv::getTickFrequency()
       << " sec");
  // Convert model to PointCLoud2 message
  // HelperFunctions::debug_print(this->pc_model);
  DrostObjectSearch::cvMat_to_pointcloud(this->pc_model, *this->msg);
  this->msg->header.stamp = ros::Time::now();
  this->msg->header.frame_id = "base_link";
}

DrostObjectSearch::~DrostObjectSearch()
{
  delete this->detector;
}

void DrostObjectSearch::pcl_callback(const sensor_msgs::PointCloud2& pcl_msg)
{
  this->pcl_sub.shutdown();
  ROS_INFO_STREAM("[DrostObjectSearch::pcl_callback()] " << "It's a point cloud");

  // Convert the scene (Adding normals)
  cv::Mat pcTest;
  if(!DrostObjectSearch::pointcloud_to_cvMat(pcl_msg, pcTest)){
    // Error during conversion
    ROS_WARN_STREAM("[DrostObjectSearch::pcl_callback()] " << "Convertion from the point cloud to scene failed [OpenCV]" );
    if(!HelperFunctions::wrapper_pcl_normals_computation(pcl_msg, pcTest)){
        // Error during conversion
        ROS_WARN_STREAM("[DrostObjectSearch::pcl_callback()] " << "Convertion from the point cloud to scene failed [PCL]" );
        //Restart subscriber
        this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &DrostObjectSearch::pcl_callback, this);
        return;
    }
  }
  ROS_DEBUG_STREAM("[DrostObjectSearch::pcl_callback()] " << "Converted point cloud to scene" );
  //pcTest = HelperFunctions::removeNaN(pcTest); //- takes very long
  //HelperFunctions::replaceNaN(pcTest, FLT_MIN);

  // Match the model to the scene and get the pose
  ROS_INFO_STREAM("[DrostObjectSearch::pcl_callback()] " << "Starting matching..." );
  vector<cv::ppf_match_3d::Pose3DPtr> results;
  int64 tick1 = cv::getTickCount();
  this->detector->match(pcTest, results, this->sm_relSceneSampleStep, this->sm_relSceneDistance);
  int64 tick2 = cv::getTickCount();

  // Some informations about the matching
  ROS_DEBUG_STREAM("[DrostObjectSearch::pcl_callback()] " << "PPF Elapsed Time " <<
       (tick2-tick1)/cv::getTickFrequency() << " sec with " << results.size() << " results");
  cv::ppf_match_3d::Pose3DPtr pose_max_votes;
  unsigned int n_votes = 0;
  if (results.size() != 0){
    for(int i = 0; i< results.size(); i++){
      //results[i]->printPose();
      if(n_votes < results[i]->numVotes){
        n_votes = results[i]->numVotes;
        pose_max_votes = results[i];
      }
    }
  }
  geometry_msgs::Pose retPose;
  // from: https://github.com/opencv/opencv_contrib/issues/464
  // You might as well simply not use ICP for this if the pose output from the detector is sufficient.
  // Another option is to use point to point ICP, since your surface normals are not describing much
  // (they are similar all over the place).

  // Create an instance of ICP
  try{
    cv::ppf_match_3d::ICP icp(this->sm_icp_iterations,    // 100
                              this->sm_icp_tolerance,     // 0.005f
                              this->sm_icp_rejectionScale,// 2.5f
                              this->sm_icp_numLevels);    // 8
    int64 t1 = cv::getTickCount();

    // Register for all selected poses
    ROS_INFO_STREAM("[DrostObjectSearch::pcl_callback()] " << "Performing ICP on " << results.size() << " poses...");
    //HelperFunctions::debug_print(this->pc_model);
    //HelperFunctions::debug_print(pcTest);
    icp.registerModelToScene(this->pc_model, pcTest, results); // fails due to bad initial pose?!
    int64 t2 = cv::getTickCount();

    ROS_INFO_STREAM("[DrostObjectSearch::pcl_callback()] " << "ICP Elapsed Time " <<
         (t2-t1)/cv::getTickFrequency() << " sec" << endl);
    //publish pose
    retPose =  HelperFunctions::toROSPose(results[0]);

//    if (ros_marker_msg.action == visualization_msgs::Marker::ADD)
//      ros_marker_msg.action = visualization_msgs::Marker::MODIFY;
  }
  catch(...){
    // When ICP fails or instead
    ROS_WARN_STREAM("[DrostObjectSearch::pcl_callback] ICP failed - publish Pose with the most votes");
    // publish Pose with the most votes
    retPose =  HelperFunctions::toROSPose(pose_max_votes);
  }
  geometry_msgs::PoseStamped pubPose;
  pubPose.pose = retPose;
  pubPose.header = pcl_msg.header;
  this->pose_pub.publish(pubPose);
  //publish model
  this->ros_model_msg.header = pcl_msg.header;
  this->ros_model_msg.mesh_poses[0] = retPose;
  this->model_pub.publish(this->ros_model_msg);
  //publish object as marker
  this->ros_marker_msg.header = pcl_msg.header;
  this->ros_marker_msg.pose = retPose;
  this->marker_pub.publish(this->ros_marker_msg);
  //Restart subscriber
  this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &DrostObjectSearch::pcl_callback, this);

}

bool DrostObjectSearch::pointcloud_to_cvMat(const sensor_msgs::PointCloud2 &point_cloud, cv::Mat& pointsAndNormals)
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
  HelperFunctions::debug_print(points, pointsAndNormals);
  ROS_DEBUG_STREAM("DrostObjectSearch::pointcloud_to_cvMat(): " << "End" << endl);
  return retValue == 1;
}

void DrostObjectSearch::cvMat_to_pointcloud(const cv::Mat &cv_pcl, sensor_msgs::PointCloud2 &msg)
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
