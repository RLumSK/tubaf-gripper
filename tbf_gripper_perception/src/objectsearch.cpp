#include "../include/tbf_gripper_perception/objectsearch.h"

ObjectSearch::ObjectSearch(ros::NodeHandle& handle,const std::string& model_name,const std::string& pcl_topic):
    nh(handle),
    pose_pub(nh.advertise<geometry_msgs::PoseStamped>("/perception/objectsearch/obj_pose", 1)),
    model_pub(nh.advertise<moveit_msgs::CollisionObject>("/perception/objectsearch/model", 1)),
    model_pcl_pub(nh.advertise<sensor_msgs::PointCloud2>("/perception/objectsearch/model_pcl", 1)),
    scene_pcl_pub(nh.advertise<sensor_msgs::PointCloud2>("/perception/objectsearch/scene_pcl", 1)),
    marker_pub(nh.advertise<visualization_msgs::Marker>("/perception/objectsearch/object", 1)),
    marker_array_pub(nh.advertise<visualization_msgs::MarkerArray>("/perception/objectsearch/pose_gueses", 1)),
    tf_listener(nh),
    msg(new sensor_msgs::PointCloud2),
    ros_model_msg(),
    ros_marker_msg(),
    ros_marker_array_msg(),
    pcl_topic(pcl_topic)
  {
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

    //load and publish model
    HelperFunctions::cvMat_to_pointcloud(cv::ppf_match_3d::loadPLYSimple((ros::package::getPath("tbf_gripper_perception") +"/meshes/" + model_name +".ply").c_str(), 1), *this->msg);
    this->msg->header.stamp = ros::Time::now();
    this->msg->header.frame_id = "gripper_camera_rgb_optical_frame";
    this->model_pcl_pub.publish(this->msg);

    ros::Duration(2.0).sleep();
    this->pcl_sub = this->nh.subscribe(pcl_topic, 1, &ObjectSearch::pcl_callback, this);
     ROS_DEBUG_STREAM("[ObjectSearch::ObjectSearch()] " << "Constructor finished" );
  }

void ObjectSearch::publish_pose(const sensor_msgs::PointCloud2& pcl_msg, const geometry_msgs::Pose& pose){
  geometry_msgs::PoseStamped pubPose;
  pubPose.pose = pose;
  pubPose.header = pcl_msg.header;
  this->pose_pub.publish(pubPose);
  //publish collision model
  this->ros_model_msg.header = pubPose.header;
  this->ros_model_msg.mesh_poses[0] = pubPose.pose;
  this->model_pub.publish(this->ros_model_msg);
  //publish object as marker
  this->ros_marker_msg.header = pubPose.header;
  this->ros_marker_msg.pose = pubPose.pose;
  this->marker_pub.publish(this->ros_marker_msg);
}

void ObjectSearch::publish_pose(const sensor_msgs::PointCloud2& pcl_msg, Eigen::Matrix4f& transformation){
    geometry_msgs::Pose pose;
    HelperFunctions::eigenMatrix4f_to_pose(transformation, pose);
    ObjectSearch::publish_pose(pcl_msg, pose);
}
void ObjectSearch::publish_pose(const sensor_msgs::PointCloud2& pcl_msg, double* affine_transformation){
    geometry_msgs::Pose pose;
    HelperFunctions::double16_to_pose(affine_transformation, pose);
    ObjectSearch::publish_pose(pcl_msg, pose);
}
