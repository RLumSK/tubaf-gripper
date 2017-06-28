#include "../include/tbf_gripper_perception/icpobjectsearch.h"

IcpObjectSearch::IcpObjectSearch(ros::NodeHandle& handle,const std::string& model_name,const std::string& pcl_topic):
  nh(handle),
  pose_pub(nh.advertise<geometry_msgs::PoseStamped>("/perception/obj_pose", 1)),
  model(cv::ppf_match_3d::loadPLYSimple((ros::package::getPath("tbf_gripper_perception") +"/meshes/" + model_name +".ply").c_str(), 1))
{
  nh.param<double>("cv_icp_tolerance", cv_icp_tolerance, 0.05);
  nh.param<double>("cv_icp_rejectionScale", cv_icp_rejectionScale, 2.5);
  nh.param<int>("cv_icp_iterations", cv_icp_iterations, 30);
  nh.param<int>("cv_icp_numLevels", cv_icp_numLevels, 30);

  // Dynamic Reconfigure
  this->f = boost::bind(&IcpObjectSearch::config_callback, this,  _1, _2);
  server.setCallback(f);

  this->cv_icp = cv::ppf_match_3d::ICP(
        cv_icp_tolerance,
        cv_icp_rejectionScale,
        cv_icp_iterations,
        cv_icp_numLevels
        );

  HelperFunctions::cvMat_to_pointcloud(this->model, model_msg);
  this->model_msg.header.stamp = ros::Time::now();
  this->model_msg.header.frame_id = "gripper_camera_rgb_optical_frame";
  this->pcl_sub = this->nh.subscribe(pcl_topic, 1, &IcpObjectSearch::pcl_callback, this);

}

void IcpObjectSearch::pcl_callback(const sensor_msgs::PointCloud2 &pcl_msg)
{
  this->pcl_sub.shutdown();

  // Convert the scene (Adding normals)
  if(!HelperFunctions::pointcloud_to_cvMat(pcl_msg, this->scene)){
    // Error during conversion
    ROS_WARN_STREAM("[IcpObjectSearch::pcl_callback()] " << "Convertion from the point cloud to scene failed [OpenCV]" );
    if(!HelperFunctions::wrapper_pcl_normals_computation(pcl_msg, this->scene)){
        // Error during conversion
        ROS_WARN_STREAM("[IcpObjectSearch::pcl_callback()] " << "Convertion from the point cloud to scene failed [PCL]" );
        //Restart subscriber
        this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &IcpObjectSearch::pcl_callback, this);
        return;
    }
  }
  HelperFunctions::debug_print(this->model);
  HelperFunctions::debug_print(this->scene);


//  int cv::ppf_match_3d::ICP::registerModelToScene	(	const Mat & 	srcPC,
//  const Mat & 	dstPC,
//  double & 	residual,
//  double 	pose[16]
//  )
  double residual;
  double pose[16];
  int retValue = this->cv_icp.registerModelToScene(
          this->model,
          this->scene,
          residual,
          pose
        );
  ROS_INFO_STREAM("[IcpObjectSearch::pcl_callback()] " << " ICP finised" << endl <<
                  "retValue: " << retValue << " Residual: "<< residual << endl <<
                  "pose:\t" << pose[0] << " "<< pose[1] << " "<< pose[2] << " "<< pose[3] << endl <<
                  "pose:\t" << pose[4] << " "<< pose[5] << " "<< pose[6] << " "<< pose[7] << endl <<
                  "pose:\t" << pose[8] << " "<< pose[9] << " "<< pose[10] << " "<< pose[11] << endl <<
                  "pose:\t" << pose[12] << " "<< pose[13] << " "<< pose[14] << " "<< pose[15] << endl
                  );

  //Restart subscriber
  this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &IcpObjectSearch::pcl_callback, this);
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
  this->pcl_sub = this->nh.subscribe(this->pcl_sub.getTopic(), 1, &IcpObjectSearch::pcl_callback, this);
}
