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
  //HelperFunctions::cvMat_to_pointcloud(this->cv_model, model_msg);

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
  ROS_INFO_STREAM("[CvIcpObjectSearch::pcl_callback()] " << " ICP finised" << endl <<
                  "retValue: " << retValue << " Residual: "<< residual << endl <<
                  "pose:\t" << pose[0] << " "<< pose[1] << " "<< pose[2] << " "<< pose[3] << endl <<
                  "pose:\t" << pose[4] << " "<< pose[5] << " "<< pose[6] << " "<< pose[7] << endl <<
                  "pose:\t" << pose[8] << " "<< pose[9] << " "<< pose[10] << " "<< pose[11] << endl <<
                  "pose:\t" << pose[12] << " "<< pose[13] << " "<< pose[14] << " "<< pose[15] << endl
                  );
  ObjectSearch::publish_pose(pcl_msg, pose);

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
