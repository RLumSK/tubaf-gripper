#ifndef PLANNINGWRAPPER_H
#define PLANNINGWRAPPER_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <eigen_conversions/eigen_msg.h>

// vgl.: http://docs.ros.org/indigo/api/moveit_tutorials/html/ and https://github.com/ros-planning/moveit_tutorials/tree/indigo-devel/doc/pr2_tutorials/planning/src

/**
 * @brief The PlanningWrapper class implements an interface to MoveIt-ROS Interfaces for the Julius robot
 */
class PlanningWrapper
{
private:
  // Setup
  ros::NodeHandle nh;
  robot_model_loader::RobotModelLoader* robot_model_loader; //("robot_description");
  robot_model::RobotModelPtr kinematic_model;              // = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene;         // (kinematic_model);
  robot_state::RobotState* current_state;                  // = planning_scene.getCurrentStateNonConst();
  std::string group_name;
  std::string ee_link;
  ros::Publisher planning_scene_diff_publisher;            // for asynchr. updates
  ros::ServiceClient planning_scene_diff_client;           // for synchr. updates
  ros::Publisher display_publisher;

  // Collision Checking
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_detection::AllowedCollisionMatrix allowed_collision_matrix;

  //Object Handling
  moveit_msgs::AttachedCollisionObject attached_object;

  // Motion Planning
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  planning_interface::MotionPlanRequest motion_request;
  planning_interface::MotionPlanResponse motion_response;
  geometry_msgs::PoseStamped target_pose;
  std::vector<double> target_joints;

  std::vector<double> tolerance_pose; //(3, 0.01);
  std::vector<double> tolerance_angle; //(3, 0.01);

protected:
  /**
   * @brief update_planning_scene_diff update the planning scene either asynchronous or synchr.
   * @param planning_scene update
   */
  void update_planning_scene_diff(moveit_msgs::PlanningScene planning_scene) const;

public:
  /**
   * @brief PlanningWrapper constructor
   */
  PlanningWrapper();

  /**
   * @brief check_self_collison checks if the robot is in self collision state
   * @return true if robot is in collision
   */
  bool check_self_collison();

  /**
   * @brief update_current_state updates the current state of the planning scene
   */
  void update_current_state();

  /**
   * @brief set_collision_request sets the internal collsion request
   * @param request collision request to check
   */
  void set_collision_request(collision_detection::CollisionRequest& request);

  /**
   * @brief get_collision_result result of the collision check
   * @return copy of the internal collision_result
   */
  collision_detection::CollisionResult get_collision_result(void) const;

  /**
   * @brief update_allowed_collision_matrix_with_current_contacts if it is certain that the robot is not in a self collision state, we can allow all current detected collsions as false positiv
   */
  void update_allowed_collision_matrix_with_current_contacts();

  /**
   * @brief check_collsion check if the robot is in collsion with itself or its environment
   * @return true if in collision
   */
  bool check_collsion();

  /**
   * @brief set_motion_request set the internal motion request
   * @param req motion request
   */
  void set_motion_request(planning_interface::MotionPlanRequest& req);

  /**
   * @brief get_motion_response get the internal motion response
   * @return copy of the current internal motion response
   */
  planning_interface::MotionPlanResponse get_motion_response(void) const;

  /**
   * @brief set_target_ps set the target pose for the motion planner
   * @param pose pose stamped
   */
  void set_target_ps(geometry_msgs::PoseStamped pose);

  /**
   * @brief plan generate a plan using the target_pose and MoveIt
   * @return true if a plan was found
   */
  bool plan_to_pose();

  /**
   * @brief set_target_joints set the target joints for the motion planner
   * @param joint_values target joints of the arm
   */
  void set_target_joints(std::vector<double> joint_values);

  /**
   * @brief plan_to_joints
   * @return  true if a plan was found
   */
  bool plan_to_joints();

  /**
   * @brief visualize_plan Visualize the plan
   */
  void visualize_plan();

  /**
   * @brief set_attached_object pass an object to be added to the end effector
   * @param obj object desired to grasp
   */
  void set_attached_object(moveit_msgs::AttachedCollisionObject obj);

  /**
   * @brief sense_object Add a prior defined object into the environment
   */
  void sense_object() const;

  /**
   * @brief attach_object attache the prior defined object
   */
  void attach_object() const;

  /**
   * @brief detach_object detach the prior attached object
   */
  void detach_object() const;

  /**
   * @brief remove_object remove the object from the entire world
   */
  void remove_object() const;
};

#endif // PLANNINGWRAPPER_H
