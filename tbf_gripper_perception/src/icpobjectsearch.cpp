#include "../include/tbf_gripper_perception/icpobjectsearch.h"

IcpObjectSearch::IcpObjectSearch(ros::NodeHandle& handle,const std::string& model_name,const std::string& pcl_topic):
  ObjectSearch(handle,  model_name, pcl_topic)
{
  ROS_DEBUG_STREAM("[IcpObjectSearch::IcpObjectSearch()] " << "Constructor called");
  nh.param<double>("icp_tolerance", icp_tolerance, 0.05);
  nh.param<double>("icp_rejectionScale", icp_rejectionScale, 2.5);
  nh.param<int>("icp_iterations", icp_iterations, 30);
  nh.param<int>("icp_numLevels", icp_numLevels, 30);

  // Dynamic Reconfigure
  this->f = boost::bind(&IcpObjectSearch::config_callback, this,  _1, _2);
  server.setCallback(f); // ERROR: calling this->unsubscribe not implemented here
  ROS_DEBUG_STREAM("[IcpObjectSearch::IcpObjectSearch()] " << "Constructor finsihed");
}


void IcpObjectSearch::config_callback(tbf_gripper_perception::IcpMatchingConfig &config, uint32_t level)
{
  //this->unsubscribe();
  this->icp_iterations = config.cv_icp_iterations;
  this->icp_tolerance = config.cv_icp_tolerance;
  this->icp_rejectionScale = config.cv_icp_rejectionScale;
  this->icp_numLevels = config.cv_icp_numLevels;

  //Restart subscriber
  //this->subscribe();
}
