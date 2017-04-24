#include "planningwrapper.h"

void wait_for_input(void){
  std::cout << "Press Enter to Continue";
  std::cin.ignore();
}

int main(int argc, char ** argv){
  std::cout << "Starting ... " << std::endl;
  // Set up ROS.
  ros::init(argc, argv, "planning");
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }
  // Create a new PlanningWrapper object.
  PlanningWrapper pl_wrap;
  collision_detection::CollisionRequest req;
  req.group_name = "UR5";

  //Self-Collsion
  pl_wrap.set_collision_request(req);
  pl_wrap.update_current_state();
  while(pl_wrap.check_self_collison()){
    ROS_INFO("including collisions in acm");
    pl_wrap.update_allowed_collision_matrix_with_current_contacts();
    pl_wrap.update_current_state();
  }

  //Define the attached object message
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "object_link";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "odom";
  /* The id of the object */
  attached_object.object.id = "box";

  /* A default pose */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x += -1.0;
  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.075;
  primitive.dimensions[1] = 0.075;
  primitive.dimensions[2] = 0.075;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);
  attached_object.touch_links = pl_wrap.get_end_effector_links();
  //attached_object.detach_posture = // trajectory_msgs/JointTrajectory
  //attached_object.weight = // float

  pl_wrap.set_attached_object(attached_object);
  pl_wrap.sense_object();
  wait_for_input();
  pl_wrap.attach_object();
  wait_for_input();
  pl_wrap.detach_object();
  wait_for_input();
  pl_wrap.remove_object();
  wait_for_input();

  //Set target pose
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = attached_object.object.header.frame_id;
  ps.pose.position.z += 10.0;
  pl_wrap.set_target_ps(ps);
  ROS_INFO_STREAM("planning.main() Finished");

}

