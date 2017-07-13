#ifndef CVICPOBJECTSEARCH_H
#define CVICPOBJECTSEARCH_H

#include <tbf_gripper_perception/icpobjectsearch.h>

class CvIcpObjectSearch : public IcpObjectSearch
{
private:
  cv::Mat cv_model, cv_scene;
  cv::ppf_match_3d::ICP cv_icp;

protected:
  void subscribe() override;
  void unsubscribe() override;

public:
  CvIcpObjectSearch(ros::NodeHandle& handle,const std::string& model_name,const std::string& pcl_topic);

   /** Callback of the Pointcloud Subscriber with the current scene
   * @brief pcl_callback current scene as point cloud
   * @param pcl_msg sensor_msgs::PointCloud2
   */
  virtual void pcl_callback(const sensor_msgs::PointCloud2& pcl_msg);

  /** Pass parameter changes to the node; see dynamic_reconfigure tutorials
   * @brief config_callback Change parameters
   * @param config new cofiguration
   * @param level level, which is the result of ORing together all of level values of the parameters that have changed
   */
  virtual void config_callback(tbf_gripper_perception::IcpMatchingConfig &config, uint32_t level);

};

#endif // CVICPOBJECTSEARCH_H
