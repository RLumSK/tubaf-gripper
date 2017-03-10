#include "planningwrapper.h"

int main(int argc, char ** argv){
  std::cout << "Starting ... " << std::endl;
  // Set up ROS.
  ros::init(argc, argv, "planning");
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }
  ROS_INFO_STREAM("planning.main() running 1");
  std::cout << "test";
  // Create a new PlanningWrapper object.
  PlanningWrapper pl_wrap;
  collision_detection::CollisionRequest req;
  req.group_name = "UR5";
  ROS_INFO_STREAM("planning.main() running 2");
  std::cout << "test";

  //Self-Collsion
  pl_wrap.set_collision_request(req);
  pl_wrap.update_current_state();
  while(!pl_wrap.check_self_collison()){
    ROS_INFO("including collisions in acm");
    pl_wrap.update_allowed_collision_matrix_with_current_contacts();
    pl_wrap.update_current_state();
  }
  ROS_INFO_STREAM("planning.main() running 3");

  //Define the attached object message
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "object_link";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "gripper_ee_link";
  /* The id of the object */
  attached_object.object.id = "box";

  ROS_INFO_STREAM("planning.main() running 4");
  /* A default pose */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.z += 10.0;
  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  ROS_INFO_STREAM("planning.main() running 5");
  pl_wrap.set_attached_object(attached_object);
  pl_wrap.sense_object();
  pl_wrap.attach_object();
  pl_wrap.detach_object();
  pl_wrap.remove_object();

  //Set target pose
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = attached_object.object.header.frame_id;
  ps.pose.position.z += 10.0;
  pl_wrap.set_target_ps(ps);
  ROS_INFO_STREAM("planning.main() Finished");

}
