#include "../include/planningwrapper.h"

PlanningWrapper::PlanningWrapper():
  planner_plugin_loader(),
  planner_instance(),
  // Tolerances for the target pose during motion planning
  tolerance_pose(3, 0.01),
  tolerance_angle(3, 0.01)
{
  ROS_DEBUG_STREAM("PlanningWrapper::PlanningWrapper constructor started");

  //getting parameters
  this->nh = ros::NodeHandle("~");
  std::string robot_description, group_name, ee_link, planner_plugin_name, name_space;
  this->nh.param("robot_description", robot_description, std::string("robot_description"));
  this->nh.param("group_name", group_name, std::string("UR5"));
  this->nh.param("ee_link", ee_link, std::string("gripper_ee_link"));
  this->nh.param("planner_plugin_name", planner_plugin_name, std::string("ompl_interface/OMPLPlanner"));
  this->nh.param("name_space", name_space, std::string(""));
  ROS_DEBUG_STREAM("PlanningWrapper::PlanningWrapper paramter imported");


  this->robot_model_loader = new robot_model_loader::RobotModelLoader(robot_description);
  this->kinematic_model = this->robot_model_loader->getModel();
  this->planning_scene = boost::make_shared<planning_scene::PlanningScene>(this->kinematic_model);
  this->update_current_state();
  this->group_name = group_name;
  this->ee_link = ee_link;
  this->allowed_collision_matrix = this->planning_scene->getAllowedCollisionMatrix();

  ROS_DEBUG_STREAM("PlanningWrapper::PlanningWrapper: Starting pluginlib class loading");
  try
  {
    this->planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("PlanningWrapper::PlanningWrapper: Exception while creating planning plugin loader " << ex.what());
  }
  try
  {

    this->planner_instance.reset(this->planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!this->planner_instance->initialize(this->kinematic_model, name_space))
      ROS_FATAL_STREAM("PlanningWrapper::PlanningWrapper:Could not initialize planner instance");
    ROS_INFO_STREAM("PlanningWrapper::PlanningWrapper: Using planning interface '" << this->planner_instance->getDescription() << "'");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string> &classes = this->planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0 ; i < classes.size() ; ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("PlanningWrapper::PlanningWrapper: Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
  }
  ROS_DEBUG_STREAM("PlanningWrapper::PlanningWrapper: Finished pluginlib class loading");

  this->planning_scene_diff_publisher = this->nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  this->planning_scene_diff_client = this->nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  this->planning_scene_diff_client.waitForExistence();

  this->display_publisher = this->nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);


  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(1.0);
  sleep_time.sleep();
  ROS_DEBUG_STREAM("PlanningWrapper::PlanningWrapper constructor finished");
}

// Setter & Getter

void PlanningWrapper::set_collision_request(collision_detection::CollisionRequest &request){
    this->collision_request = request;
    ROS_DEBUG_STREAM("PlanningWrapper::set_collision_request() finished");
}

collision_detection::CollisionResult PlanningWrapper::get_collision_result() const{
  ROS_DEBUG_STREAM("PlanningWrapper::get_collision_result()");
  return this->collision_result;
}

void PlanningWrapper::set_motion_request(planning_interface::MotionPlanRequest &request){
    this->motion_request = request;
    ROS_DEBUG_STREAM("PlanningWrapper::set_motion_request() finished");
}

planning_interface::MotionPlanResponse PlanningWrapper::get_motion_response() const{
  ROS_DEBUG_STREAM("PlanningWrapper::get_motion_response()");
  return this->motion_response;
}

void PlanningWrapper::set_target_ps(geometry_msgs::PoseStamped pose){
    this->target_pose = pose;
    ROS_DEBUG_STREAM("PlanningWrapper::set_target_ps() finished");
}

void PlanningWrapper::set_target_joints(std::vector<double> joint_values){
    if(joint_values.size() != 6){
      ROS_WARN_STREAM("PlanningWrapper::set_target_joints() illegal joint_values.size() == " << joint_values.size());
      return;
    }
    this->target_joints = joint_values;
    ROS_DEBUG_STREAM("PlanningWrapper::set_target_joints() finished");
}

void PlanningWrapper::set_attached_object(moveit_msgs::AttachedCollisionObject obj){
    this->attached_object = obj;
    ROS_DEBUG_STREAM("PlanningWrapper::set_attached_object() finished");
}

// Updates & Checks

bool PlanningWrapper::check_self_collison(){
  ROS_DEBUG_STREAM("PlanningWrapper::check_self_collison() started");
  this->collision_result.clear();
  this->planning_scene->checkSelfCollision(this->collision_request, this->collision_result, this->planning_scene->getCurrentState(), this->allowed_collision_matrix);
  ROS_INFO_STREAM("PlanningWrapper::check_self_collison: Current state is " << (this->collision_result.collision ? "in" : "not in") << " self collision");
  ROS_DEBUG_STREAM("PlanningWrapper::check_self_collison() finished");
  return this->collision_result.collision;
}

void PlanningWrapper::update_current_state(){
  this->current_state = &this->planning_scene->getCurrentStateNonConst();
  ROS_DEBUG_STREAM("PlanningWrapper::update_current_state() finished");
}

void PlanningWrapper::update_allowed_collision_matrix_with_current_contacts(){
  ROS_DEBUG_STREAM("PlanningWrapper::update_allowed_collision_matrix_with_current_contacts() started");
  this->allowed_collision_matrix = this->planning_scene->getAllowedCollisionMatrix();
  robot_state::RobotState copied_state = this->planning_scene->getCurrentState();

  collision_detection::CollisionResult::ContactMap::const_iterator it2;
  for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2){
    this->allowed_collision_matrix.setEntry(it2->first.first, it2->first.second, true);
    ROS_DEBUG_STREAM("PlanningWrapper::update_allowed_collision_matrix_with_current_contacts() allow " << it2->first.first << " - " << it2->first.second << " collsion");

  }
  ROS_DEBUG_STREAM("PlanningWrapper::update_allowed_collision_matrix_with_current_contacts() finished");
}

bool PlanningWrapper::check_collsion(){
  ROS_DEBUG_STREAM("PlanningWrapper::check_collsion() started");
  this->collision_result.clear();
  this->planning_scene->checkCollision(this->collision_request, this->collision_result, this->planning_scene->getCurrentState(), this->allowed_collision_matrix);
  ROS_INFO_STREAM("PlanningWrapper::check_collsion(): Current state is " << (collision_result.collision ? "in" : "not in") << " collision");
  return collision_result.collision;
}

void PlanningWrapper::update_planning_scene_diff(moveit_msgs::PlanningScene planning_scene) const{
  //asynchr.
  this->planning_scene_diff_publisher.publish(planning_scene);
//synchr.
//  moveit_msgs::ApplyPlanningScene srv;
//  srv.request.scene = planning_scene;
//  planning_scene_diff_client.call(srv);
}

// Planning & Actions
bool PlanningWrapper::plan_to_pose(){
  ROS_DEBUG_STREAM("PlanningWrapper::plan_to_pose() started");
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(this->ee_link, this->target_pose, this->tolerance_pose, this->tolerance_angle);
  this->motion_request.goal_constraints.push_back(pose_goal);
  planning_interface::PlanningContextPtr context = this->planner_instance->getPlanningContext(this->planning_scene, this->motion_request, this->motion_response.error_code_);
  context->solve(this->motion_response);
  if(this->motion_response.error_code_.val != this->motion_response.error_code_.SUCCESS){
    ROS_ERROR_STREAM("PlanningWrapper::plan_to_pose() Could not compute plan successfully -  error code: " << this->motion_response.error_code_.val);
    return false;
  }
  ROS_DEBUG_STREAM("PlanningWrapper::plan_to_pose() finished");
  return true;
}

bool PlanningWrapper::plan_to_joints(){
  moveit_msgs::MotionPlanResponse response;
  this->motion_response.getMessage(response);
  const robot_state::JointModelGroup* joint_model_group = this->current_state->getJointModelGroup(this->group_name);
  this->current_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
}

void PlanningWrapper::sense_object() const{
  ROS_INFO("PlanningWrapper::sense_object(): Adding the object into the world");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  this->update_planning_scene_diff(planning_scene);
  ROS_INFO("PlanningWrapper::sense_object(): finished");
}

void PlanningWrapper::attach_object() const{
  ROS_INFO("PlanningWrapper::attach_object(): started");
//Attaching an object requires two operations
//  - Removing the original object from the environment
//  - Attaching the object to the robot
  /* First, define the REMOVE object message*/
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = this->attached_object.object.id;
  remove_object.header.frame_id = this->attached_object.link_name;
  remove_object.operation = moveit_msgs::CollisionObject::REMOVE;
  /* Carry out the REMOVE + ATTACH operation */
  ROS_INFO("PlanningWrapper::attach_object(): Attaching the object to the robot and removing it from the world.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.robot_state.attached_collision_objects.push_back(this->attached_object);

  this->update_planning_scene_diff(planning_scene);
  ROS_INFO("PlanningWrapper::attach_object(): finished");
}

void PlanningWrapper::detach_object() const{
  ROS_INFO("PlanningWrapper::detach_object(): started");
//Detaching an object from the robot requires two operations
// - Detaching the object from the robot
// - Re-introducing the object into the environment
  /* First, define the DETACH object message*/
  moveit_msgs::AttachedCollisionObject detach_object;
  detach_object.object.id = this->attached_object.object.id;
  detach_object.link_name = "object_link";
  detach_object.object.operation = moveit_msgs::CollisionObject::REMOVE;
  /* Carry out the DETACH + ADD operation */
  ROS_INFO("PlanningWrapper::detach_object(): Detaching the object from the robot and returning it to the world.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_object.object);

  this->update_planning_scene_diff(planning_scene);
  ROS_INFO("PlanningWrapper::detach_object(): finished");
}

void PlanningWrapper::remove_object() const{
  ROS_INFO("PlanningWrapper::remove_object(): started");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();

  moveit_msgs::CollisionObject remove_object;
  remove_object.id = this->attached_object.object.id;
  remove_object.header.frame_id = this->attached_object.link_name;
  remove_object.operation = moveit_msgs::CollisionObject::REMOVE;

  planning_scene.world.collision_objects.push_back(remove_object);
  this->update_planning_scene_diff(planning_scene);
  ROS_INFO("PlanningWrapper::remove_object(): started");
}

//Visualization

void PlanningWrapper::visualize_plan(){
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  ROS_INFO("PlanningWrapper::visualize_plan() Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  this->motion_response.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  this->display_publisher.publish(display_trajectory);
}
