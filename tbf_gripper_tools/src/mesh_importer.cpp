#include "ros/ros.h"
#include "shape_msgs/Mesh.h"

#include <geometric_shapes/shape_operations.h>

int main (int argc, char ** argv){
  ros::init(argc, argv, "wlan_box_mesh_publisher");
  ros::NodeHandle node_handle("~");

  std::string mesh_path, mesh_name;
  if(node_handle.getParam("mesh_path", mesh_path)){
    ros::param::param<std::string>("~mesh_name", mesh_name, "no_mesh_name");
    // http://answers.ros.org/question/245995/adding-collision-object-in-moveit/
    shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);
    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);

    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    //publish mesage at mesh_name-Topic
    ros::Publisher pub = node_handle.advertise<shape_msgs::Mesh>(mesh_name, 10);
    ros::Rate loop_rate(10);
    while(ros::ok()){
      pub.publish(mesh);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  else{
    ROS_WARN("mesh_importer.cpp: No mesh_path set");
    return -1;
  }

  return 0;
}
