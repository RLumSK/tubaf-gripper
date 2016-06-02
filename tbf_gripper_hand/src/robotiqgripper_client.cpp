#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tbf_gripper_hand/RobotiqGripperAction.h>

using namespace tbf_gripper_hand;
typedef actionlib::SimpleActionClient<RobotiqGripperAction> Client;

class RobotiqActionClientNode
{
public:
    RobotiqActionClientNode(): ac("robotiqgripper_action_server", true){
        ROS_INFO("RobotiqActionClientNode(): Waiting for action server to start.");
        // wait for the action server to start
        ac.waitForServer(); //will wait for infinite time
        ROS_INFO("RobotiqActionClientNode(): Action server started, sending goal.");
    }

    void start(int position, int speed=110, int force=111){
        RobotiqGripperActionGoal goal;
        goal.mode = "basic";
        goal.position = position;
        goal.speed = speed;
        goal.force = force;
        ac.sendGoal(goal,
                    boost::bind(&RobotiqActionClientNode::doneCallback, this, _1, _2),
                    Client::SimpleActiveCallback(),
                    boost::bind(&RobotiqActionClientNode::feedbackCallback, this, _1, _2)
                    );
    }

    void doneCallback(const actionlib::SimpleClientGoalState& state, const RobotiqGripperActionResultPtr& result){
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("Answer: %s", result->result.hand_info.c_str());
        ros::shutdown();
    }

    void feedbackCallback(const actionlib::SimpleClientGoalState& state, const RobotiqGripperActionFeedbackPtr& feedback){
        ROS_INFO("Goal in state [%s]", state.toString().c_str());
        ROS_INFO("Answer: GTO = %i", feedback->feedback.hand_status.gGTO);
        ros::shutdown();
    }
private:
    Client ac;
};


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_robotiqgripper");
  RobotiqActionClientNode my_node;
  my_node.start(255);
  ros::spin();
  return 0;
}
