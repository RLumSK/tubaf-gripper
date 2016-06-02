#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <tbf_gripper_hand/RobotiqGripperAction.h>

#include <stdlib.h>

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

    void start(int position, int speed=0, int force=111){
		RobotiqGripperGoal goal;
        goal.mode = "basic";
        goal.position = position;
        goal.speed = speed;
        goal.force = force;
        ac.sendGoal(goal,
					boost::bind(&RobotiqActionClientNode::doneCallback, this, _1, _2),
                    Client::SimpleActiveCallback(),
					boost::bind(&RobotiqActionClientNode::feedbackCallback, this, _1)
                    );
    }

	void doneCallback(const actionlib::SimpleClientGoalState& state,  const RobotiqGripperResultConstPtr& result){
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ROS_INFO("Answer: %s", result->hand_info.c_str());
        //ros::shutdown();
        ros::Duration(3).sleep();
        this->start(rand() % 255 + 1);
    }

	void feedbackCallback(const RobotiqGripperFeedbackConstPtr& feedback){
//        ROS_INFO("Goal in state [%s]", state.toString().c_str());
        ROS_INFO("Feedback: GTO = %i", feedback->hand_status.gGTO);
    }
private:
    Client ac;
};


int main (int argc, char **argv)
{
  srand(0);
  ros::init(argc, argv, "test_robotiqgripper");
  RobotiqActionClientNode my_node;
  my_node.start(rand() % 255 + 1);
  ros::spin();
  return 0;
}
