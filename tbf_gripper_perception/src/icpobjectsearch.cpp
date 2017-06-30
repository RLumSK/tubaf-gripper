#include "../include/tbf_gripper_perception/icpobjectsearch.h"

IcpObjectSearch::IcpObjectSearch(ros::NodeHandle& handle,const std::string& model_name,const std::string& pcl_topic):
  nh(handle),
  pose_pub(nh.advertise<geometry_msgs::PoseStamped>("/perception/obj_pose", 1)),
  marker_pub(nh.advertise<visualization_msgs::Marker>("/perception/object", 1)),
  cv_model(cv::ppf_match_3d::loadPLYSimple((ros::package::getPath("tbf_gripper_perception") +"/meshes/" + model_name +".ply").c_str(), 1)),
  pcl_model(new pcl::PointCloud<pcl::PointXYZ>),
  pcl_scene(new pcl::PointCloud<pcl::PointXYZ>),
  pcl_icp(),
  ros_marker_msg()
{
  nh.param<double>("cv_icp_tolerance", cv_icp_tolerance, 0.05);
  nh.param<double>("cv_icp_rejectionScale", cv_icp_rejectionScale, 2.5);
  nh.param<int>("cv_icp_iterations", cv_icp_iterations, 30);
  nh.param<int>("cv_icp_numLevels", cv_icp_numLevels, 30);

  // Dynamic Reconfigure
  this->f = boost::bind(&IcpObjectSearch::config_callback, this,  _1, _2);
  server.setCallback(f);

  // OpenCV
  this->cv_icp = cv::ppf_match_3d::ICP(
        cv_icp_tolerance,
        cv_icp_rejectionScale,
        cv_icp_iterations,
        cv_icp_numLevels
        );
  HelperFunctions::cvMat_to_pointcloud(this->cv_model, model_msg);
  this->model_msg.header.stamp = ros::Time::now();
  this->model_msg.header.frame_id = "gripper_camera_rgb_optical_frame";

  // PCL
  pcl::PLYReader reader;
  pcl::PCLPointCloud2 cloud;
  reader.read((ros::package::getPath("tbf_gripper_perception") +"/meshes/" + model_name +".ply").c_str(), cloud);
  pcl::fromPCLPointCloud2(cloud, *this->pcl_model);
  this->pcl_icp.setInputSource(this->pcl_model);

  // Marker to visualize estimate
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

  //this->pcl_sub = this->nh.subscribe(pcl_topic, 1, &IcpObjectSearch::cv_icp_callback, this);
  this->pcl_sub = this->nh.subscribe(pcl_topic, 1, &IcpObjectSearch::pcl_icp_callback, this);

}

void IcpObjectSearch::pcl_icp_callback(const sensor_msgs::PointCloud2ConstPtr &pcl_msg)
{
  if(pcl_msg->data.empty())
  {
    ROS_WARN_STREAM("[IcpObjectSearch::pcl_icp_callback()] " << "Received an empty cloud message. Skipping further processing");
    return;
  }
  this->pcl_sub.shutdown();

  // Convert ROS message to PCL-compatible data structure - see: https://github.com/aleksandaratanasov/pmd_camboard_nano/blob/master/src/pmd_camboard_nano_cloud_icp.cpp
  ROS_INFO_STREAM("[IcpObjectSearch::pcl_icp_callback()] " << "Received a cloud message with " << pcl_msg->height * pcl_msg->width << " points");
  ROS_INFO_STREAM("[IcpObjectSearch::pcl_icp_callback()] " << "Converting ROS cloud message to PCL compatible data structure");
  pcl::fromROSMsg(*pcl_msg, *this->pcl_scene);

  this->pcl_icp.setMaximumIterations(this->cv_icp_iterations);

  this->pcl_icp.setInputTarget(this->pcl_scene);

  pcl::PointCloud<pcl::PointXYZ> final;
  this->pcl_icp.align(final);
  ROS_INFO_STREAM("[IcpObjectSearch::pcl_icp_callback()] " << "has converged:" << this->pcl_icp.hasConverged() << " score: " << this->pcl_icp.getFitnessScore());
  Eigen::Matrix4f transformation = this->pcl_icp.getFinalTransformation();
  ROS_INFO_STREAM("[IcpObjectSearch::pcl_icp_callback()] " << " ICP finised" << endl <<
                  "transformation:\t" << transformation(0,0) << " "<< transformation(0,0) << " "<< transformation(0,2) << " "<< transformation(0,3) << endl <<
                  "transformation:\t" << transformation(1,0) << " "<< transformation(1,1) << " "<< transformation(1,2) << " "<< transformation(1,3) << endl <<
                  "transformation:\t" << transformation(2,0) << " "<< transformation(2,1) << " "<< transformation(2,2) << " "<< transformation(2,3) << endl <<
                  "transformation:\t" << transformation(3,0) << " "<< transformation(3,1) << " "<< transformation(3,2) << " "<< transformation(3,3) << endl
                  );
  geometry_msgs::PoseStamped ps_msg;
  ps_msg.header = pcl_msg->header;
  HelperFunctions::eigenMatrix4f_to_pose(transformation, ps_msg.pose);
  this->pose_pub.publish(ps_msg);

  this->ros_marker_msg.header = ps_msg.header;
  this->ros_marker_msg.pose = ps_msg.pose;
  this->marker_pub.publish(this->ros_marker_msg);

  //Restart subscriber
  this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &IcpObjectSearch::pcl_icp_callback, this);
}

void IcpObjectSearch::cv_icp_callback(const sensor_msgs::PointCloud2ConstPtr &pcl_msg)
{
  this->pcl_sub.shutdown();

  // Convert the scene (Adding normals)
  if(!HelperFunctions::pointcloud_to_cvMat(*pcl_msg, this->cv_scene)){
    // Error during conversion
    ROS_WARN_STREAM("[IcpObjectSearch::cv_icp_callback()] " << "Convertion from the point cloud to scene failed [OpenCV]" );
    if(!HelperFunctions::wrapper_pcl_normals_computation(*pcl_msg, this->cv_scene)){
        // Error during conversion
        ROS_WARN_STREAM("[IcpObjectSearch::cv_icp_callback()] " << "Convertion from the point cloud to scene failed [PCL]" );
        //Restart subscriber
        this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &IcpObjectSearch::cv_icp_callback, this);
        return;
    }
  }
  HelperFunctions::debug_print(this->cv_model);
  HelperFunctions::debug_print(this->cv_scene);


//  int cv::ppf_match_3d::ICP::registerModelToScene	(	const Mat & 	srcPC,
//  const Mat & 	dstPC,
//  double & 	residual,
//  double 	pose[16]
//  )
  double residual;
  double pose[16];
  int retValue = this->cv_icp.registerModelToScene(
          this->cv_model,
          this->cv_scene,
          residual,
          pose
        );
  ROS_INFO_STREAM("[IcpObjectSearch::cv_icp_callback()] " << " ICP finised" << endl <<
                  "retValue: " << retValue << " Residual: "<< residual << endl <<
                  "pose:\t" << pose[0] << " "<< pose[1] << " "<< pose[2] << " "<< pose[3] << endl <<
                  "pose:\t" << pose[4] << " "<< pose[5] << " "<< pose[6] << " "<< pose[7] << endl <<
                  "pose:\t" << pose[8] << " "<< pose[9] << " "<< pose[10] << " "<< pose[11] << endl <<
                  "pose:\t" << pose[12] << " "<< pose[13] << " "<< pose[14] << " "<< pose[15] << endl
                  );
  geometry_msgs::PoseStamped ps_msg;
  ps_msg.header = pcl_msg->header;
  HelperFunctions::double16_to_pose(pose, ps_msg.pose);
  this->pose_pub.publish(ps_msg);

  //Restart subscriber
  this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &IcpObjectSearch::cv_icp_callback, this);
}

void IcpObjectSearch::config_callback(tbf_gripper_perception::IcpMatchingConfig &config, uint32_t level)
{
  this->pcl_sub.shutdown();
  this->cv_icp_iterations = config.cv_icp_iterations;
  this->cv_icp_tolerance = config.cv_icp_tolerance;
  this->cv_icp_rejectionScale = config.cv_icp_rejectionScale;
  this->cv_icp_numLevels = config.cv_icp_numLevels;

  this->cv_icp = cv::ppf_match_3d::ICP(
        cv_icp_tolerance,
        cv_icp_rejectionScale,
        cv_icp_iterations,
        cv_icp_numLevels
        );

  //Restart subscriber
  //this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &IcpObjectSearch::cv_icp_callback, this);
  this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &IcpObjectSearch::pcl_icp_callback, this);
}
