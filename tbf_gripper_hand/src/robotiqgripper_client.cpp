#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <control_msgs/GripperCommandAction.h>

#include <stdlib.h>
#include <random>


class RobotiqActionClientNode
{
private:
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac;
    // https://stackoverflow.com/questions/686353/random-float-number-generation
    std::random_device rd;
    std::mt19937 e2;
    std::uniform_real_distribution<float> dist;
    bool request_send;

public:
    RobotiqActionClientNode(std::string as_name):
      e2(rd()),
      dist(0.0, 0.16),
      request_send(false),
      ac(as_name, true){
        ROS_INFO_STREAM("[RobotiqActionClientNode] Waiting for action server "<< as_name <<" to start.");
        // wait for the action server to start
        ac.waitForServer(); //will wait for infinite time
        ROS_INFO_STREAM("[RobotiqActionClientNode] Action server "<< as_name <<" started, sending goal.");
    }

    void start(float position, float force=20.0){
//        if(!this->request_send){
//          return;
//        }
        control_msgs::GripperCommandGoal goal;
        goal.command.position = position;
        goal.command.max_effort = force;
        this->request_send = true;
        ac.sendGoal(goal,
					boost::bind(&RobotiqActionClientNode::doneCallback, this, _1, _2),
                    actionlib::SimpleActionClient<control_msgs::GripperCommandAction>::SimpleActiveCallback(),
					boost::bind(&RobotiqActionClientNode::feedbackCallback, this, _1)
                    );
    }

  void doneCallback(const actionlib::SimpleClientGoalState& state,  const control_msgs::GripperCommandResultConstPtr& result){
        ROS_INFO("[RobotiqActionClientNode] Finished in state [%s]", state.toString().c_str());
        ROS_INFO("[RobotiqActionClientNode] Answer: \n\t Position: %g\n\t Effort: %g\n\t "
                 "Stalled: %s\n\t Reached: %s",
                 result->position, result->effort, result->stalled ? "true" : "false", result->reached_goal ? "true" : "false");
       this->request_send = false;
    }

  void setRandomPosition(){
      //  https://stackoverflow.com/questions/686353/random-float-number-generation
      float r = this->dist(this->e2);
      ROS_INFO("Set Position: %g", r);
      this->start(r);
  }

  void feedbackCallback(const control_msgs::GripperCommandFeedbackConstPtr & feedback){
        ROS_DEBUG("[RobotiqActionClientNode] Feedback: \n\t Position: %g\n\t Effort: %g\n\t "
                 "Stalled: %s\n\t Reached: %s",
                 feedback->position, feedback->effort, feedback->stalled ? "true" : "false", feedback->reached_goal ? "true" : "false");
    }
};


int main (int argc, char **argv)
{
  ros::init(argc, argv, "gripper_action_client", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  std::string as_name;
  nh.param<std::string>("gripper_action_server_name", as_name, "Robotiq3FGripperServer");
  RobotiqActionClientNode my_node(as_name);
  ros::Duration pause(5, 0);
  while(ros::ok()){
    my_node.setRandomPosition();
    pause.sleep();
  }
  return 0;
}
