/*****************************************************************
 * SPIN THE BOTTLE GAME: By Diego Alejandro Gómez Pardo.		 *
 *																 *
 *  Laboratory in Humanoid Robotics								 *
 *  Humanoid Robots Group										 *
 *  Computer Siences Department 4								 *
 *	Rheinische Friedrich Wilhelms Universität Bonn				 *
 *																 *
 *	ActionClientSay node: This node is the one used in order to  *
 *	make nao say some of the words used to reestablish the normal*
 *	flow of the game (put bottle in the middle, no bottle, etc)	 *
 *****************************************************************/

#ifndef ac_say_def
#define ac_say_def
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <naoqi_msgs/SpeechWithFeedbackAction.h>
#include <naoqi_msgs/SpeechWithFeedbackGoal.h>
#include <string>
namespace ac_say
{
	class ActionClientSay
	{
	private:
		actionlib::SimpleActionClient<naoqi_msgs::SpeechWithFeedbackAction> ac;
	public:
		ActionClientSay() : ac("/speech_action", true)
	{
			ROS_INFO("Waiting for action server to start : '/speech_action'");
			while(!ac.waitForServer(ros::Duration(1.0))) {
				ROS_INFO("Waiting ...");
				ros::spinOnce();
			}
	}
		int say(std::string toSay)
		{
			{
				ROS_INFO_STREAM ("Debug: Invoked ActionClientSay:: Say" );
				naoqi_msgs::SpeechWithFeedbackGoal goal;
				goal.say = toSay;
				ac.sendGoal(goal);
				ROS_INFO_STREAM ("Debug: goal sent" );
				bool finishedBeforeTimeout = ac.waitForResult(ros::Duration(2.5));
				if(finishedBeforeTimeout)
				{
					actionlib::SimpleClientGoalState state = ac.getState();
					ROS_INFO("Action finished: %s", state.toString().c_str());
				}
				else
					ROS_INFO("Action did not finish before the time out.");
				return 0;
			}
		}
	};
}// end namespace ac_say

#endif  // ac_say_def
