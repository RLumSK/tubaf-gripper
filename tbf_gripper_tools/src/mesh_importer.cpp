#include "ros/ros.h"
#include "shape_msgs/Mesh.h"

#include <geometric_shapes/shape_operations.h>
#include <XmlRpcValue.h>

shape_msgs::Mesh create_mesh_msg(std::string path){
  // http://answers.ros.org/question/245995/adding-collision-object-in-moveit/
  shapes::Mesh* m = shapes::createMeshFromResource(path);
  m->scale(0.1);
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);

  return boost::get<shape_msgs::Mesh>(mesh_msg);
}


int main (int argc, char ** argv){
  ros::init(argc, argv, "wlan_box_mesh_publisher");
  ros::NodeHandle node_handle("~");
  ros::NodeHandle gl_nh;

  XmlRpc::XmlRpcValue name_param, mesh_param;
  std::string mesh_path, mesh_name;
  std::vector<ros::Publisher> pubs;
  std::vector<shape_msgs::Mesh> meshes;
  node_handle.getParam("mesh_name", name_param);
  node_handle.getParam("mesh_path", mesh_param);

  if(name_param.getType() == XmlRpc::XmlRpcValue::TypeString){
    mesh_name = static_cast<std::string>(name_param);
    mesh_path = static_cast<std::string>(mesh_param);
    pubs.push_back(gl_nh.advertise<shape_msgs::Mesh>(std::string("/")+mesh_name, 10));
    meshes.push_back(create_mesh_msg(mesh_path));
  }
  else if (name_param.getType() == XmlRpc::XmlRpcValue::TypeArray){
    for (int32_t i = 0; i < name_param.size(); ++i){
     mesh_name = static_cast<std::string>(name_param[i]);
     mesh_path = static_cast<std::string>(mesh_param[i]);
     pubs.push_back(node_handle.advertise<shape_msgs::Mesh>(std::string("/")+mesh_name, 10));
     meshes.push_back(create_mesh_msg(mesh_path));
    }
  }
  else{
    ROS_WARN("mesh_importer.cpp: mesh_name is not of type Array or String");
    return -1;
  }

  ros::Rate loop_rate(10);
  while(ros::ok()){
    for(int i = 0; i<pubs.size(); i++)
      pubs[i].publish(meshes[i]);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
