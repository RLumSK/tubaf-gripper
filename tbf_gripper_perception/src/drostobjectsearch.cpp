#include "tbf_gripper_perception/drostobjectsearch.h"

using namespace std;

DrostObjectSearch::DrostObjectSearch(ros::NodeHandle& handle, const string& model_name, const  string& pcl_topic):
  nh(handle),
  pose_pub(nh.advertise<geometry_msgs::PoseStamped>("/perception/obj_pose", 1)),
  model_pub(nh.advertise<moveit_msgs::CollisionObject>("/perception/model", 1)),
  model_pcl_pub(nh.advertise<sensor_msgs::PointCloud2>("/perception/model_pcl", 1)),
  scene_pcl_pub(nh.advertise<sensor_msgs::PointCloud2>("/perception/scene_pcl", 1)),
  marker_pub(nh.advertise<visualization_msgs::Marker>("/perception/object", 1)),
  marker_array_pub(nh.advertise<visualization_msgs::MarkerArray>("/perception/pose_gueses", 1)),
  tf_listener(nh),
  msg(new sensor_msgs::PointCloud2),
  pc_model(cv::ppf_match_3d::loadPLYSimple((ros::package::getPath("tbf_gripper_perception") +"/meshes/" + model_name +".ply").c_str(), 1)),
  ros_model_msg(),
  ros_marker_msg(),
  ros_marker_array_msg()
{
  nh.param<double>("sm_relSampleStep", sm_relSampleStep, 0.5);
  nh.param<double>("sm_relDistanceStep", sm_relDistanceStep, 0.05);
  nh.param<double>("sm_relSceneSampleStep", sm_relSceneSampleStep, 0.2);
  nh.param<double>("sm_relSceneDistance", sm_relSceneDistance, 0.03);
  nh.param<double>("cv_icp_tolerance", sm_icp_tolerance, 0.05);
  nh.param<double>("cv_icp_rejectionScale", sm_icp_rejectionScale, 2.5);
  nh.param<int>("sm_numAngles", sm_numAngles, 30);
  nh.param<int>("cv_icp_iterations", sm_icp_iterations, 30);
  nh.param<int>("cv_icp_numLevels", sm_icp_numLevels, 30);
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
  dummy_marker = ros_marker_msg;
  dummy_marker.ns = "obj_pose_guess";
  dummy_marker.type = visualization_msgs::Marker::ARROW;
  dummy_marker.scale.x = 0.1;
  dummy_marker.scale.y = 0.01;
  dummy_marker.scale.z = 0.01;
  dummy_marker.color.r = 1.0;
  dummy_marker.color.g = 0.0;
  dummy_marker.color.b = 1.0;

  // Dynamic Reconfigure
  this->f = boost::bind(&DrostObjectSearch::config_callback, this,  _1, _2);
  server.setCallback(f);
  //server.setCallback(this->config_callback);

  // Surface Matching
  // Now train the model
  this->trainDetector();

  this->model_pcl_pub.publish(this->msg);

  ros::Duration(2.0).sleep();
  this->pcl_sub = this->nh.subscribe(pcl_topic, 1, &DrostObjectSearch::pcl_callback, this);
  ROS_DEBUG_STREAM("[DrostObjectSearch::DrostObjectSearch()] " << "Constructor finished" );
}

DrostObjectSearch::~DrostObjectSearch()
{
  delete this->detector;
}

void DrostObjectSearch::pcl_callback(const sensor_msgs::PointCloud2& pcl_msg)
{
  this->pcl_sub.shutdown();
  ROS_INFO_STREAM("[DrostObjectSearch::pcl_callback()] " << "It's a point cloud");
  ros::Duration(2.0).sleep(); // get TransformListener rdy

  // Convert scene-pointcloud to "base_link" for propper matching
  sensor_msgs::PointCloud2 cloud_out;
  std::string target_link = "base_link";
  std::string source_link = pcl_msg.header.frame_id;
  std::string error_msg;
  while(!this->tf_listener.waitForTransform(target_link, source_link, pcl_msg.header.stamp, ros::Duration(1.0))){
    ROS_WARN_STREAM("[DrostObjectSearch::pcl_callback()] " << "No transform from: " << source_link << " to " << target_link);
  }
  pcl_ros::transformPointCloud(target_link, pcl_msg, cloud_out, this->tf_listener);
  // Publish current model and scene for debugging in rviz
  this->model_pcl_pub.publish(this->msg);
  this->scene_pcl_pub.publish(cloud_out);

  // Convert the scene (Adding normals)
  cv::Mat pcTest;
  if(!HelperFunctions::pointcloud_to_cvMat(cloud_out, pcTest)){
    // Error during conversion
    ROS_WARN_STREAM("[DrostObjectSearch::pcl_callback()] " << "Convertion from the point cloud to scene failed [OpenCV]" );
    if(!HelperFunctions::wrapper_pcl_normals_computation(cloud_out, pcTest)){
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
  ROS_INFO_STREAM("[DrostObjectSearch::pcl_callback()] " << "PPF Elapsed Time " <<
       (tick2-tick1)/cv::getTickFrequency() << " sec with " << results.size() << " results");
  cv::ppf_match_3d::Pose3DPtr pose_max_votes;
  unsigned int n_votes = 0;
  this->ros_marker_array_msg.markers.clear();
  this->dummy_marker.header = cloud_out.header;

  if (results.size() != 0){
    for(int i = 0; i< results.size(); i++){
      //results[i]->printPose();
      if(n_votes < results[i]->numVotes){
        n_votes = results[i]->numVotes;
        pose_max_votes = results[i];
      }
      this->dummy_marker.id = i;
      this->dummy_marker.pose = HelperFunctions::toROSPose(results[i]);
      this->ros_marker_array_msg.markers.push_back(this->dummy_marker);
    }
  }
  else{
    // No poses found
    ROS_INFO_STREAM("[DrostObjectSearch::pcl_callback()] " << "No poses found");//Restart subscriber
    this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &DrostObjectSearch::pcl_callback, this);
    return;
  }
  this->marker_array_pub.publish(this->ros_marker_array_msg);
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
  }
  catch(...){
    // When ICP fails or instead
    ROS_WARN_STREAM("[DrostObjectSearch::pcl_callback] ICP failed - publish Pose with the most votes");
    // publish Pose with the most votes
    retPose =  HelperFunctions::toROSPose(pose_max_votes);
  }

  geometry_msgs::PoseStamped pubPose;
  pubPose.pose = retPose;
  pubPose.header = cloud_out.header;
  this->pose_pub.publish(pubPose);
  //publish model
  this->ros_model_msg.header = cloud_out.header;
  this->ros_model_msg.mesh_poses[0] = retPose;
  this->model_pub.publish(this->ros_model_msg);
  //publish object as marker
  this->ros_marker_msg.header = cloud_out.header;
  this->ros_marker_msg.pose = retPose;
  this->marker_pub.publish(this->ros_marker_msg);

  //Restart subscriber
  this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &DrostObjectSearch::pcl_callback, this);

}

void DrostObjectSearch::trainDetector()
{
  if(this->detector != NULL)
    delete this->detector;
  this->detector = new DebugDetector(sm_relSampleStep, sm_relDistanceStep, sm_numAngles);
  ROS_DEBUG_STREAM("[DrostObjectSearch::trainDetector()] " << "Model" << endl <<
                  "Columns: " << pc_model.cols << endl <<
                  "MatSize: "<< *this->pc_model.size);
  ROS_DEBUG_STREAM("[DrostObjectSearch::trainDetector()] " << "Training...");
  int64 tick1 = cv::getTickCount();
  //this->pc_model.convertTo(this->pc_model, CV_32FC1);
  this->detector->trainModel(this->pc_model);
  int64 tick2 = cv::getTickCount();
  ROS_INFO_STREAM("[DrostObjectSearch::trainDetector()] " << "Training complete in "
       << (double)(tick2-tick1)/ cv::getTickFrequency()
       << " sec");
  // Convert model to PointCLoud2 message
  HelperFunctions::cvMat_to_pointcloud(this->detector->getSampledPC(), *this->msg);
  this->msg->header.stamp = ros::Time::now();
  this->msg->header.frame_id = "gripper_camera_rgb_optical_frame";
}

void DrostObjectSearch::config_callback(tbf_gripper_perception::SurfaceMatchingConfig &config, uint32_t level)
{
  this->pcl_sub.shutdown();
  ROS_INFO_STREAM("[DrostObjectSearch::config_callback()] " << "New parameter " );
  if(this->sm_relSampleStep != config.sm_relSampleStep || this->sm_relDistanceStep != config.sm_relDistanceStep || this->sm_numAngles != config.sm_numAngles){
    this->sm_relSampleStep = config.sm_relSampleStep;
    this->sm_relDistanceStep = config.sm_relDistanceStep;
    this->sm_numAngles = config.sm_numAngles;
    this->trainDetector();
  }
  this->sm_relSceneSampleStep= config.sm_relSceneSampleStep;
  this->sm_relSceneDistance = config.sm_relSceneDistance;
  this->sm_icp_iterations = config.sm_icp_iterations;
  this->sm_icp_tolerance = config.sm_icp_tolerance;
  this->sm_icp_rejectionScale = config.sm_icp_rejectionScale;
  this->sm_icp_numLevels = config.sm_icp_numLevels;

  //Restart subscriber
  this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &DrostObjectSearch::pcl_callback, this);
}
