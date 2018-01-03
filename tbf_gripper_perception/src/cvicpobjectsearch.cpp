#include "tbf_gripper_perception/cvicpobjectsearch.h"

CvIcpObjectSearch::CvIcpObjectSearch(ros::NodeHandle& handle,const std::string& model_name,const std::string& pcl_topic):
  IcpObjectSearch(handle,  model_name, pcl_topic)
{
  // OpenCV
  this->cv_icp = cv::ppf_match_3d::ICP(
        icp_tolerance,
        icp_rejectionScale,
        icp_iterations,
        icp_numLevels
        );
  HelperFunctions::pointcloud_to_cvMat(*this->msg,this->cv_model);

  this->subscribe();
}

void CvIcpObjectSearch::subscribe(){
  this->pcl_sub = this->nh.subscribe(pcl_topic, 1, &CvIcpObjectSearch::pcl_callback, this);
}

void CvIcpObjectSearch::unsubscribe(){
  this->pcl_sub.shutdown();
}

void CvIcpObjectSearch::pcl_callback(const sensor_msgs::PointCloud2& pcl_msg){
if(pcl_msg.data.empty())
  {
    ROS_WARN_STREAM("[CvIcpObjectSearch::pcl_callback()] " << "Received an empty cloud message. Skipping further processing");
    return;
  }
  this->unsubscribe();
    // Convert the scene (Adding normals)
  if(!HelperFunctions::pointcloud_to_cvMat(pcl_msg, this->cv_scene)){
    // Error during conversion
    ROS_WARN_STREAM("[CvIcpObjectSearch::pcl_callback()] " << "Convertion from the point cloud to scene failed [OpenCV]" );
    if(!HelperFunctions::wrapper_pcl_normals_computation(pcl_msg, this->cv_scene)){
        // Error during conversion
        ROS_WARN_STREAM("[CvIcpObjectSearch::pcl_callback()] " << "Convertion from the point cloud to scene failed [PCL]" );
        //Restart subscriber
        this->subscribe();
        return;
    }
  }
  // HelperFunctions::debug_print(this->cv_model);
  // HelperFunctions::debug_print(this->cv_scene);


//  int cv::ppf_match_3d::ICP::registerModelToScene	(	const Mat & 	srcPC,
//  const Mat & 	dstPC,
//  double & 	residual,
//  double 	pose[16]
//  )
  double residual;
  cv::Matx44d pose;
  // From: http://docs.opencv.org/trunk/dc/d9b/classcv_1_1ppf__match__3d_1_1ICP.html#accd9744cedf9cd9cd175d2c5bd77951e
  // It is assumed that the model is registered on the scene. Scene remains static, while the model transforms.
  // The output poses transform the models onto the scene. Because of the point to plane minimization,
  // the scene is expected to have the normals available. Expected to have the normals (Nx6).
  // -- On successful termination, the function returns 0.--
  int retValue = this->cv_icp.registerModelToScene(
    //[in]	srcPC	The input point cloud for the model. Expected to have the normals (Nx6). Currently, CV_32F is the only supported data type.
      this->cv_model,
    //[in]	dstPC	The input point cloud for the scene. It is assumed that the model is registered on the scene. Scene remains static.
    //Expected to have the normals (Nx6). Currently, CV_32F is the only supported data type.
      this->cv_scene,
    //[out]	residual	The output registration error.
      residual,
    //[out]	pose	Transformation between srcPC and dstPC.
      pose
         );


//  ROS_INFO_STREAM("[CvIcpObjectSearch::pcl_callback()] " << " ICP finised" << endl <<
//                  "retValue: " << retValue << " Residual: " << residual << endl <<
//                  "pose:\t" << pose.val[0, 0] << " "<< pose.val[0, 1] << " "<< pose.val[0, 2] << " "<< pose.val[0, 3] << endl <<
//                  "pose:\t" << pose.val[1, 0] << " "<< pose.val[1, 1] << " "<< pose.val[1, 2] << " "<< pose.val[1, 3] << endl <<
//                  "pose:\t" << pose.val[2, 0] << " "<< pose.val[2, 1] << " "<< pose.val[2, 2] << " "<< pose.val[2, 3] << endl <<
//                  "pose:\t" << pose.val[3, 0] << " "<< pose.val[3, 1] << " "<< pose.val[3, 2] << " "<< pose.val[3, 3] << endl
//                  );

  ObjectSearch::publish_pose(pcl_msg, pose, true);

  //Restart subscriber
  this->subscribe();
}

void CvIcpObjectSearch::config_callback(tbf_gripper_perception::IcpMatchingConfig &config, uint32_t level)
{
  IcpObjectSearch::config_callback(config, level);

  this->cv_icp = cv::ppf_match_3d::ICP(
        icp_tolerance,
        icp_rejectionScale,
        icp_iterations,
        icp_numLevels
        );

}
