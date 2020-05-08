#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>

//Robotiq messages
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput.h>

#include <boost/lexical_cast.hpp>
#include <sstream>
#include <string>

inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}


class RobotiqGripperActionServer
{
private:
	ros::Publisher gripper_pub;
	ros::Subscriber gripper_sub;
	bool lock = false;
	bool working = false;


	/**
	 * parse the hand status to a string according to the manual
	 * http://support.robotiq.com/pages/viewpage.action?pageId=590045
	 * @param in message from the hand (looking for gIMC here)
	 * @return string with the current hand status
	 */
	std::string getHandStatus(robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput in){
		switch(in.gIMC){
		case 0: return "0 - reset";
		case 1: return "1 - activating";
		case 2: return "2 - changing mode";
		case 3: return "3 - completed";
		default: return "";
		}
	}

	/**
	 * Initialize the Robotiq S Model gripper
	 * http://support.robotiq.com/pages/viewpage.action?pageId=590044
	 * @return message that will initialize the gripper
	 */
	static robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput initGripper(int mode = 0){
		robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput msg;
		msg.rACT = 1; // Activate Gripper (must stay on after activation routine is completed).
		msg.rMOD = mode; // 0 - Basic, 1 -Pinch, 2 - Wide, 3 - Scissor
		msg.rATR = 0; // 0 - Normal, 1 - Emergency Release
		msg.rGLV = 1; // glove enabled
		msg.rGTO = 0; // goto requested position
		msg.rICF = 0; // Individual Control of Fingers Mode (disabled)
		msg.rICS = 0; // Individual Control of Scissor (disabled)
		msg.rPRA =10; // Position; 0 - open, 255 - close
		msg.rSPA =10; // Speed: 0 - 22mm/s, 255 - 110 mm/s, dv/drSPA = 0.34mm/s
		msg.rFRA =10; // Force: 0 - 15N, 255 - 60N, dF/dFRA = 0.175N (approximate value, relation non-linear)
		// other fingers are not used just set the values to the same of finger A
		msg.rPRB = msg.rPRA;
		msg.rPRC = msg.rPRA;
		msg.rSPB = msg.rSPA;
		msg.rSPC = msg.rSPA;
		msg.rFRB = msg.rFRA;
		msg.rFRC = msg.rFRA;
		// set scissor to zero, shouldn't matter as individual scissor control is disabled
		msg.rPRS = 0;
		msg.rSPS = 0;
		msg.rFRS = 0;
		return msg;
	}

	/**
	 * parse the mode from the given string to an unsigned int for the message
	 * @param mode string [basic, pinch, wide, scissor] - wide is not recommended in gripper mode
	 * @return value [0, 1, 2, 3], unknown will return 0
	 */
	static uint8_t parseMode(std::string mode){
		if(mode == "basic"){
			return 0;
		}
		else if(mode == "pinch"){
			return 1;
		}
		else if(mode == "wide"){
			return 2;
		}
		else if(mode == "scissor"){
			return 3;
		}
		else{
			return 0;
		}
	}

	/**
   * @brief calculate the value to be set to achieve a given speed
	 * @param speed speed in mm/s [22, 110]
	 * @return value between 0-255 matching the given speed
	 */
	static uint8_t calcSpeed(float speed){
		float tmp = (speed - 23.0) / 0.34;
		if(tmp > 255.0){
			return 255;
		}
		else if(tmp < 0){
			return 0;
		}
		else{
			return (uint8_t)tmp;
		}
	}

	/**
   * @brief approximate the value to be set to achieve a given force (relation non-linear)
	 * @param force force in N [15, 60]
	 * @return value between 0-255 matching the given force
	 */
	static uint8_t approximateForce(float force){
		float tmp = (force - 16.0) / 0.175;
		if(tmp > 255.0){
			return 255;
		}
		else if(tmp < 0){
			return 0;
		}
		else{
			return (uint8_t)tmp;
		}
	}

  /**
   * @brief calcGapFromMsg calculate the current gripper positon a Robotiq3FGripperRobotInput message
   * @param msg message from the Robotiq3FGripper
   * @return current gap size in meters
   */
  float calcGapFromMsg(){
    return (1.0-static_cast<float>(this->msg_from_gripper->gPOA)/255.0)*0.16;  //gPOA : Returns the actual position
  }

  /**
   * @brief calcEffortFromMsg calculate the current gripper effort a Robotiq3FGripperRobotInput message
   * @param msg message from the Robotiq3FGripper
   * @return current effort in Newtons
   */
  float calcEffortFromMsg(){
    return static_cast<float>(this->msg_from_gripper->gCUA)*0.175+15.0;  //gCUA : Returns a value that represents current consumption
  }

  /**
   * @brief setStalledFromMsg set if the current gripper is stalled from Robotiq3FGripperRobotInput message
   * @param msg message from the Robotiq3FGripper
   * @return current stalled status
   */
  bool setStalledFromMsg(){
//# gSTA : Motion status, returns the current motion of the Gripper fingers.
//# 0x00 - Gripper is in motion towards requested position (only meaningful if gGTO = 1)
//# 0x01 - Gripper is stopped. One or two fingers stopped before requested position
//# 0x02 - Gripper is stopped. All fingers stopped before requested position
//# 0x03 - Gripper is stopped. All fingers reached requested position
    return this->msg_from_gripper->gSTA !=0 && this->msg_from_gripper->gSTA !=3;
  }

  /**
   * @brief setGoalReachedFromMsg set if the current gripper is stalled from Robotiq3FGripperRobotInput message
   * @param msg message from the Robotiq3FGripper
   * @return current goal status
   */
  bool setGoalReachedFromMsg(){
//# gSTA : Motion status, returns the current motion of the Gripper fingers.
//# 0x00 - Gripper is in motion towards requested position (only meaningful if gGTO = 1)
//# 0x01 - Gripper is stopped. One or two fingers stopped before requested position
//# 0x02 - Gripper is stopped. All fingers stopped before requested position
//# 0x03 - Gripper is stopped. All fingers reached requested position
    return this->msg_from_gripper->gSTA == 3;
  }

    /**
     * @brief Generates the hand status fom a Robotiq3FGripperRobotInput Message
     * @return String with a formated description of the current hand status
     */
	std::string generateHandStatus(){
		const robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInputConstPtr msg = this->msg_from_gripper;
		std::stringstream ss;
		// gACT : Initialization status, echo of the rACT bit (activation bit).
		ss << "gACT = ";
		switch(msg->gACT){
		case 0:
			ss << "0 - Gripper reset.";
			break;
		case 1:
			ss << "1 - Gripper activation.";
			break;
		default:;
		}
		ss << "\n";

		// gMOD : Operation Mode status, echo of the rMOD bits (grasping mode requested).
		ss << "gMOD = ";
		switch(msg->gMOD){
		case 0:
			ss << "0 - Basic mode.";
			break;
		case 1:
			ss << "1 - Pinch mode.";
			break;
		case 2:
			ss << "2 - Wide mode.";
			break;
		case 3:
			ss << "3 - Scissor mode.";
			break;
		default:;
		}
		ss << "\n";

		// gGTO : Action status, echo of the rGTO bit (go to bit).
		ss << "gGTO = ";
		switch(msg->gGTO){
		case 0:
			ss << "0 - Stopped (or performing activation / grasping mode change / automatic release).";
			break;
		case 1:
			ss << "1 - Go to Position Request.";
			break;
		default:;
		}
		ss << "\n";


		//gIMC : Gripper status, returns the current status of the Gripper.
		ss << "gIMC = ";
		switch(msg->gIMC){
		case 0:
			ss << "0 - Gripper is in reset (or automatic release) state. See Fault status if Gripper is activated.";
			break;
		case 1:
			ss << "1 - Activation is in progress.";
			break;
		case 2:
			ss << "2 - Mode change is in progress.";
			break;
		case 3:
			ss << "3 - Activation and Mode change are complete.";
			break;
		default:;
		}
		ss << "\n";

		// gSTA : Gripper status, returns the current status & motion of the Gripper fingers.
		ss << "gSTA = ";
		switch(msg->gSTA){
		case 0:
			ss << "0 - Gripper is in motion towards requested position (only meaningful if gGTO = 1).";
			break;
		case 1:
			ss << "1 - Gripper is stopped. One or two fingers stopped before requested position.";
			break;
		case 2:
			ss << "2 - Gripper is stopped. All fingers stopped before requested position.";
			break;
		case 3:
			ss << "3 - Gripper is stopped. All fingers reached requested position.";
			break;
		default:;
		}
		ss << "\n";

		// gDTA
		ss << "gDTA = ";
		switch(msg->gDTA){
		case 0:
			ss << "0 - Finger A is in motion (only meaningful if gGTO = 1). ";
			break;
		case 1:
			ss << "1 - Finger A has stopped due to a contact while opening. ";
			break;
		case 2:
			ss << "2 - Finger A has stopped due to a contact while closing.";
			break;
		case 3:
			ss << "3 - Finger A is at the requested position.";
			break;
		default:;
		}
		ss << "\n";

		// gDTB
		ss << "gDTB = ";
		switch(msg->gDTB){
		case 0:
			ss << "0 - Finger B is in motion (only meaningful if gGTO = 1). ";
			break;
		case 1:
			ss << "1 - Finger B has stopped due to a contact while opening. ";
			break;
		case 2:
			ss << "2 - Finger B has stopped due to a contact while closing.";
			break;
		case 3:
			ss << "3 - Finger B is at the requested position.";
			break;
		default:;
		}
		ss << "\n";

		// gDTC
		ss << "gDTC = ";
		switch(msg->gDTC){
		case 0:
			ss << "0 - Finger C is in motion (only meaningful if gGTO = 1). ";
			break;
		case 1:
			ss << "1 - Finger C has stopped due to a contact while opening. ";
			break;
		case 2:
			ss << "2 - Finger C has stopped due to a contact while closing.";
			break;
		case 3:
			ss << "3 - Finger C is at the requested position.";
			break;
		default:;
		}
		ss << "\n";

		// gDTS
		ss << "gDTS = ";
		switch(msg->gDTS){
		case 0:
			ss << "0 - Scissor is in motion (only meaningful if gGTO = 1).";
			break;
		case 1:
			ss << "1 - Scissor has stopped due to a contact while opening.";
			break;
		case 2:
			ss << "2 - Scissor has stopped due to a contact while closing.";
			break;
		case 3:
			ss << "3 - Scissor is at the requested position.";
			break;
		default:;
		}
		ss << "\n";

		// gFLT : Fault status returns general error messages useful for troubleshooting.
		ss << "gFLT = ";
		switch(msg->gFLT){
		case 0:
			ss << "0 -  No fault (fault LED off)";
			break;
		case 5:
			ss << "Priority faults: 5 - Action delayed, activation (reactivation) must be completed prior to renewed action.";
			break;
		case 6:
			ss << "Priority faults: 6 - Action delayed, mode change must be completed prior to continuing action.";
			break;
		case 7:
			ss << "Priority faults: 7 - The activation bit must be set prior to action.";
			break;
		case 9:
			ss << "Minor faults: 9 - The communication chip is not ready (may be booting).";
			break;
		case 10:
			ss << "Minor faults: 10- Changing mode fault, interference detected on Scissor (for less than 20 sec).";
			break;
		case 11:
			ss << "Minor faults: 11- Automatic release in progress.";
			break;
		case 13:
			ss << "Major faults: 13- Activation fault, verify that no interference or other error occurred.";
			break;
		case 14:
			ss << "Major faults: 14- Changing mode fault, interference detected on Scissor (for more than 20 sec).";
			break;
		case 15:
			ss << "Major faults: 15- Automatic release completed. Reset and activation is required.";
			break;
		default:;
		}
        ss << "\n";
		ss << "Position information (request, actual) | current consumption:\n-------------------------------\n";
        ss << "Finger A: " << unsigned(msg->gPRA) << ", " << unsigned(msg->gPOA) << "| " << unsigned(msg->gCUA) *0.1 <<" mA\n";
        ss << "Finger B: " << unsigned(msg->gPRB) << ", " << unsigned(msg->gPOB) << "| " << unsigned(msg->gCUB) *0.1 <<" mA\n";
        ss << "Finger C: " << unsigned(msg->gPRC) << ", " << unsigned(msg->gPOC) << "| " << unsigned(msg->gCUC) *0.1 <<" mA\n";
        ss << "Scissor : " << unsigned(msg->gPRS) << ", " << unsigned(msg->gPOS) << "| " << unsigned(msg->gCUS) *0.1 <<" mA\n";
		ss << "\n";

		return ss.str();
	}

  /**
     * @brief set the necessary bits in the gripper message to achieve the given goal
   */
  void transcriptGoal(const control_msgs::GripperCommandGoalConstPtr &goal){
    this->msg_to_gripper.rGTO = 1;
    this->msg_to_gripper.rSPA = 255;
    this->msg_to_gripper.rPRA = static_cast<int>((1.0-goal->command.position/0.16)*255.0);
    this->msg_to_gripper.rFRA = static_cast<int>((goal->command.max_effort-15.0)/0.175);

    if(this->msg_to_gripper.rPRA > 255){
      this->msg_to_gripper.rPRA = 255;
    }
    else if(this->msg_to_gripper.rPRA < 0){
      this->msg_to_gripper.rPRA = 0;
    }

    if(this->msg_to_gripper.rFRA > 255){
      this->msg_to_gripper.rFRA = 255;
    }
    else if(this->msg_to_gripper.rFRA < 0){
      this->msg_to_gripper.rFRA = 0;
    }

  }

	/**
     * @brief Determine whether the gripper completed its task or not
	 * @return true if the gripper is still running
	 */
	bool checkStatus(){
        if(this->msg_from_gripper->gGTO == 0){
            return false;
        }
        bool isRunning = this->msg_from_gripper->gSTA == 3 ? true : false;	// Gripper in Motion
		return isRunning;
	}

protected:

  ros::NodeHandle nh_;
  ros::NodeHandle p_nh;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  control_msgs::GripperCommandFeedback feedback_;
  control_msgs::GripperCommandResult result_;
  robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInputConstPtr msg_from_gripper;
  robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput msg_to_gripper;
  bool isRunning = false;

public:

  RobotiqGripperActionServer(std::string name, int mode = 0) :
    p_nh("~"), /* init private node handle*/
    as_(nh_, name, boost::bind(&RobotiqGripperActionServer::executeCB, this, _1), false),
    action_name_(name)
  {

    ROS_INFO_STREAM("[RobotiqGripperActionServer] Action server '" << name << "' starting.");
    as_.start();

    std::string publisher_name = "Robotiq3FGripperRobotOutput", subscriber_name = "Robotiq3FGripperRobotInput";
    p_nh.param<std::string>("publisher_name", publisher_name, "/Robotiq3FGripperRobotOutput");
    p_nh.param<std::string>("subscriber_name", subscriber_name, "/Robotiq3FGripperRobotOutput");
    ROS_INFO_STREAM("[RobotiqGripperActionServer] Subscribe to: " << subscriber_name << "\tPublish at: " << publisher_name);
    this->gripper_pub = nh_.advertise<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput>(publisher_name, 5);

    // http://answers.ros.org/question/108551/using-subscribercallback-function-inside-of-a-class-c/
    this->gripper_sub = nh_.subscribe(subscriber_name, 5, &RobotiqGripperActionServer::onNewGripperState, this);

    ros::Duration nap(0.5);
    nap.sleep();

    this->msg_from_gripper = robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInputConstPtr(new robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput());
    this->msg_to_gripper = RobotiqGripperActionServer::initGripper(mode);
    this->gripper_pub.publish(this->msg_to_gripper);
    ROS_INFO_STREAM("[RobotiqGripperActionServer] Action server '" << name << "' started.");
  }

  ~RobotiqGripperActionServer(void)
  {
	  ROS_INFO("RobotiqGripperAction: Action server shutting down.");
	  this->gripper_pub.shutdown();
	  this->gripper_sub.shutdown();
  }

  void onNewGripperState(const robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInputConstPtr& msg){
    if(lock == true){
  		return;
  	}
  	this->lock = true;
    ROS_DEBUG("[RobotiqGripperActionServer] RobotiqGripperAction.onNewGripperState: Received new message from gripper.");
    this->msg_from_gripper = msg;
    this->feedback_.position = this->calcGapFromMsg();      // The current gripper gap size (in meters)
    this->feedback_.effort = this->calcEffortFromMsg();        // The current effort exerted (in Newtons)
    this->feedback_.stalled = this->setStalledFromMsg();       // True if the gripper is exerting max effort and not moving
    this->feedback_.reached_goal = this->setGoalReachedFromMsg();  // # True iff the gripper position has reached the commanded setpoint
  	this->lock = false;
  }

  void executeCB(const control_msgs::GripperCommandGoalConstPtr &goal)
  {
    ROS_DEBUG("[RobotiqGripperActionServer] RobotiqGripperAction.executeCB: Received new goal.");
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // start executing the action
    this->transcriptGoal(goal);
    this->feedback_.reached_goal = false;
    this->gripper_pub.publish(this->msg_to_gripper);
    // feedback-loop
    while(this->feedback_.reached_goal && !as_.isPreemptRequested() && ros::ok()){
      ROS_DEBUG("[RobotiqGripperActionServer] RobotiqGripperAction.executeCB: is running");
      as_.publishFeedback(this->feedback_);
    	r.sleep();
    }
    success = !this->isRunning;

    this->result_.position = this->calcGapFromMsg();         // The current gripper gap size (in meters)
    this->result_.effort = this->calcEffortFromMsg();        // The current effort exerted (in Newtons)
    this->result_.stalled = this->setStalledFromMsg();       // True if the gripper is exerting max effort and not moving
    this->result_.reached_goal = this->setGoalReachedFromMsg();  // # True iff the gripper position has reached the commanded setpoint

    if(as_.isPreemptRequested()){
        ROS_INFO("[RobotiqGripperActionServer] RobotiqGripperAction.executeCB(): %s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted(this->result_);
        return;
    }
    if(success)
    {
      as_.setSucceeded(this->result_);
    }
    else{
      as_.setAborted(this->result_);
    }
    ROS_INFO("[RobotiqGripperActionServer] RobotiqGripperAction.executeCB: successful achieved goal? %s", BoolToString(success));
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_action_server", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  std::string as_name;
  nh.param<std::string>("gripper_action_server_name", as_name, "Robotiq3FGripperServer");
  RobotiqGripperActionServer action_server(as_name);
  RobotiqGripperActionServer basic_as_server(as_name+"_basic", 0);
  RobotiqGripperActionServer pinch_as_server(as_name+"_pinch", 1);
  RobotiqGripperActionServer wide_as_server(as_name+"_wide", 2);
  RobotiqGripperActionServer scissor_server(as_name+"_scissor", 3);
  ros::spin();

  return 0;
}
