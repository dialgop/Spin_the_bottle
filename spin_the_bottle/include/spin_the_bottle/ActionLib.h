#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <naoqi_msgs/JointTrajectoryGoal.h>
#include <naoqi_msgs/JointTrajectoryAction.h>

class actionLib
{
private:
public:
	actionlib::SimpleActionClient<naoqi_msgs::JointTrajectoryAction> ac;
	actionLib() : ac("/joint_trajectory", true)
	{
		ROS_INFO("Waiting for action server to start.");
		while(!ac.waitForServer(ros::Duration(1.0))) {
			ROS_INFO("Waiting ...");
			ros::spinOnce();
		}
	}
	int perform(naoqi_msgs::JointTrajectoryGoal goal)
	{
		ac.sendGoal(goal);
		bool finishedBeforeTimeout = ac.waitForResult(ros::Duration(15.0));
		if(finishedBeforeTimeout)
		{
			actionlib::SimpleClientGoalState state = ac.getState();
			ROS_INFO("Action finished: %s", state.toString().c_str());
		}
		else
			ROS_INFO("Action did not finish before the time out.");
		return 0;
	}
};
