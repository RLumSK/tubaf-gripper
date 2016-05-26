#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tbf_gripper_hand/RobotiqGripperAction.h>

//Robotiq messages
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <robotiq_s_model_control/SModel_robot_input.h>

#include <boost/lexical_cast.hpp>
#include <sstream>
#include <string>

class RobotiqGripperAction
{
private:
	ros::Publisher gripper_pub;
	ros::Subscriber gripper_sub;
	bool lock = false;

	/**
	 * parse the hand status to a string according to the manual
	 * http://support.robotiq.com/pages/viewpage.action?pageId=590045
	 * @param in message from the hand (looking for gIMC here)
	 * @return string with the current hand status
	 */
	std::string getHandStatus(robotiq_s_model_control::SModel_robot_input in){
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
	static robotiq_s_model_control::SModel_robot_output initGripper(int mode = 0){
		robotiq_s_model_control::SModel_robot_output msg;
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
	 * calculate the value to be set to achieve a given speed
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
	 * approximate the value to be set to achieve a given force (relation non-linear)
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

	std::string generateHandStatus(){
		const robotiq_s_model_control::SModel_robot_inputConstPtr msg = this->msg_from_gripper;
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
		ss << "Finger A: " << boost::lexical_cast<uint8_t>(msg->gPRA) << ", " << boost::lexical_cast<uint8_t>(msg->gPOA) << "| " << boost::lexical_cast<uint8_t>(msg->gCUA) <<"\n";
		ss << "Finger B: " << boost::lexical_cast<uint8_t>(msg->gPRB) << ", " << boost::lexical_cast<uint8_t>(msg->gPOB) << "| " << boost::lexical_cast<uint8_t>(msg->gCUB) <<"\n";
		ss << "Finger C: " << boost::lexical_cast<uint8_t>(msg->gPRC) << ", " << boost::lexical_cast<uint8_t>(msg->gPOC) << "| " << boost::lexical_cast<uint8_t>(msg->gCUC) <<"\n";
		ss << "Scissor : " << boost::lexical_cast<uint8_t>(msg->gPRS) << ", " << boost::lexical_cast<uint8_t>(msg->gPOS) << "| " << boost::lexical_cast<uint8_t>(msg->gCUS) <<"\n";
		ss << "\n";

		return ss.str();
	}

	/**
	 * set the necessary bits in the gripper message to achieve the given goal
	 */
	void transcriptGoal(const tbf_gripper_hand::RobotiqGripperGoalConstPtr &goal){
		this->msg_to_gripper.rGTO = 1;
		this->msg_to_gripper.rMOD = RobotiqGripperAction::parseMode(goal->mode);
		this->msg_to_gripper.rPRA = goal->position;
		this->msg_to_gripper.rSPA = RobotiqGripperAction::calcSpeed(goal->speed);
		this->msg_to_gripper.rFRA = RobotiqGripperAction::approximateForce(goal->force);

	}

	/**
	 * Determine whether the gripper completed its task or not
	 * @param goal
	 * @return true if the gripper is still running
	 */
	bool checkStatus(const tbf_gripper_hand::RobotiqGripperGoalConstPtr &goal){
		bool isRunning = this->msg_from_gripper->gSTA==0?true:false;	// Gripper in Motion
		return isRunning;
	}

	/**
	 * Determine whether the gripper completed its task successful or not
	 * @param goal
	 * @return true if the gripper successful reached its goal
	 */
	bool checkResult(const tbf_gripper_hand::RobotiqGripperGoalConstPtr &goal){
		bool succed =  this->msg_from_gripper->gFLT==0?true:false;	// Gripper not in fault status
		return succed;
	}

protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<tbf_gripper_hand::RobotiqGripperAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  tbf_gripper_hand::RobotiqGripperFeedback feedback_;
  tbf_gripper_hand::RobotiqGripperResult result_;
  robotiq_s_model_control::SModel_robot_inputConstPtr msg_from_gripper;
  robotiq_s_model_control::SModel_robot_output msg_to_gripper;

public:

  RobotiqGripperAction(std::string name) :
    as_(nh_, name, boost::bind(&RobotiqGripperAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    // TODO: read topic names from config file / parameter server

    this->gripper_pub = nh_.advertise<robotiq_s_model_control::SModel_robot_input>("robotiq_gripper_action_server", 5);
    // http://answers.ros.org/question/108551/using-subscribercallback-function-inside-of-a-class-c/
    this->gripper_sub = nh_.subscribe("SModelRobotInput", 5, &RobotiqGripperAction::onNewGripperState, this);

    ros::Duration nap(0.5);
    nap.sleep();
    this->msg_to_gripper = RobotiqGripperAction::initGripper();
  }

  ~RobotiqGripperAction(void)
  {
	  this->gripper_pub.shutdown();
	  this->gripper_sub.shutdown();
  }

  void onNewGripperState(const robotiq_s_model_control::SModel_robot_inputConstPtr& msg){
  	if(lock == true){
  		return;
  	}
  	this->lock = true;
  	this->msg_from_gripper = msg;
	this->feedback_.hand_status = *msg;
  	this->lock = false;
  }

  void executeCB(const tbf_gripper_hand::RobotiqGripperGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // feedback initialization
	/*
	 * #feedback
	 * robotiq_s_model_control/SModel_robot_input hand_status
	 */
    // start executing the action
    bool isRunning = true;

    this->transcriptGoal(goal);
    this->gripper_pub.publish(this->msg_to_gripper);
    // feedback-loop
    while(isRunning){
    	isRunning = this->checkStatus(goal);
    	as_.publishFeedback(feedback_);
    	r.sleep();
    }

    /*
     * #result definition
     * robotiq_s_model_control/SModel_robot_input hand_status hand_status
     */
    result_.hand_status = *(this->msg_from_gripper.get());
    result_.hand_info = RobotiqGripperAction::generateHandStatus();
    if(success)
    {
    	as_.setSucceeded(result_);
    }
    else{
    	as_.setAborted(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotiq_gripper_action_server");

  RobotiqGripperAction action_server(ros::this_node::getName());
  ros::spin();

  return 0;
}
