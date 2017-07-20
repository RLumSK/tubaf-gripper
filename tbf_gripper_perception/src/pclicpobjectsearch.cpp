#include <tbf_gripper_perception/pclicpobjectsearch.h>

PclIcpObjectSearch::PclIcpObjectSearch(ros::NodeHandle& handle,const std::string& model_name,const std::string& pcl_topic):
  IcpObjectSearch(handle,  model_name, pcl_topic),
  pcl_model(new  pcl::PointCloud<pcl::PointXYZ>),
  pcl_scene(new  pcl::PointCloud<pcl::PointXYZ>)
{ // PCL
  ROS_DEBUG_STREAM("[PclIcpObjectSearch::PclIcpObjectSearch()] " << "Constructor called");
  pcl::PLYReader reader;
  pcl::PCLPointCloud2 cloud;
  reader.read((ros::package::getPath("tbf_gripper_perception") +"/meshes/" + model_name +".ply").c_str(), cloud);
  pcl::fromPCLPointCloud2<pcl::PointXYZ>(cloud, *this->pcl_model); //Assertion error
  this->pcl_icp.setInputSource(this->pcl_model);
  this->subscribe();
  ROS_DEBUG_STREAM("[PclIcpObjectSearch::PclIcpObjectSearch()] " << "Constructor finished");
}

void PclIcpObjectSearch::subscribe(){
  this->pcl_sub = this->nh.subscribe(pcl_topic, 1, &PclIcpObjectSearch::pcl_callback, this);
}

void PclIcpObjectSearch::unsubscribe(){
  this->pcl_sub.shutdown();
}

void PclIcpObjectSearch::pcl_callback(const sensor_msgs::PointCloud2& pcl_msg){
  ROS_DEBUG_STREAM("[PclIcpObjectSearch::pcl_callback()] " << "started");
  if(pcl_msg.data.empty())
  {
    ROS_WARN_STREAM("[PclIcpObjectSearch::pcl_callback()] " << "Received an empty cloud message. Skipping further processing");
    return;
  }
  this->unsubscribe();
   // Convert ROS message to PCL-compatible data structure - see: https://github.com/aleksandaratanasov/pmd_camboard_nano/blob/master/src/pmd_camboard_nano_cloud_icp.cpp
  ROS_INFO_STREAM("[PclIcpObjectSearch::pcl_callback()] " << "Received a cloud message with " << pcl_msg.height * pcl_msg.width << " points");
  ROS_DEBUG_STREAM("[PclIcpObjectSearch::pcl_callback()] " << "Converting ROS cloud message to PCL compatible data structure");
  pcl::fromROSMsg(pcl_msg, *this->pcl_scene);

  this->pcl_icp.setMaximumIterations(this->icp_iterations);

  this->pcl_icp.setInputTarget(this->pcl_scene);

  pcl::PointCloud<pcl::PointXYZ> final;
  this->pcl_icp.align(final);
  ROS_INFO_STREAM("[PclIcpObjectSearch::pcl_callback()] " << "has converged:" << this->pcl_icp.hasConverged() << " score: " << this->pcl_icp.getFitnessScore());
  Eigen::Matrix4f transformation = this->pcl_icp.getFinalTransformation();
  ROS_INFO_STREAM("[PclIcpObjectSearch::pcl_callback()] " << " ICP finised" << endl <<
                  "transformation:\t" << transformation(0,0) << " "<< transformation(0,0) << " "<< transformation(0,2) << " "<< transformation(0,3) << endl <<
                  "transformation:\t" << transformation(1,0) << " "<< transformation(1,1) << " "<< transformation(1,2) << " "<< transformation(1,3) << endl <<
                  "transformation:\t" << transformation(2,0) << " "<< transformation(2,1) << " "<< transformation(2,2) << " "<< transformation(2,3) << endl <<
                  "transformation:\t" << transformation(3,0) << " "<< transformation(3,1) << " "<< transformation(3,2) << " "<< transformation(3,3) << endl
                  );
  ObjectSearch::publish_pose(pcl_msg, transformation);


  //Restart subscriber
  this->subscribe();
  ROS_INFO_STREAM("[PclIcpObjectSearch::pcl_callback()] " << "finished");

}
