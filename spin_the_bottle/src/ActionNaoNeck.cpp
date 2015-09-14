/*****************************************************************
 * SPIN THE BOTTLE GAME: By Diego Alejandro Gómez Pardo.		 *
 *																 *
 *  Laboratory in Humanoid Robotics								 *
 *  Humanoid Robots Group										 *
 *  Computer Siences Department 4								 *
 *	Rheinische Friedrich Wilhelms Universität Bonn				 *
 *																 *
 *	ActionNaoNeck node: This is a ROS node which contains the    *
 *	information used in order to make nao turn its neck in pitch *
 *	and yaw angles. there was not enough time to connect this	 *
 *	node to the subscriber node.				 			  	 *
 *****************************************************************/

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <naoqi_msgs/JointTrajectoryGoal.h>
#include <naoqi_msgs/JointTrajectoryAction.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

class ActionNaoNeck
    {
    private:
        actionlib::SimpleActionClient<naoqi_msgs::JointTrajectoryAction> ac;
        vector<double> pitchAndYaw;
    public:
        ActionNaoNeck() : ac("/joint_trajectory", true)
		{
            ROS_INFO("Waiting for action server to start.");
            while(!ac.waitForServer(ros::Duration(1.0)))
            {
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
                return 1;
            }
            else
                ROS_INFO("Action did not finish before the time out.");
            return 0;
        }
        void setPitchAndYaw(vector<double> incomingPitchAndYaw)
        {
        	pitchAndYaw = incomingPitchAndYaw;
        }
    };

//First function --> NAO Move head down

    naoqi_msgs::JointTrajectoryGoal moveHeadDown()
    {
    	naoqi_msgs::JointTrajectoryGoal goal;
		goal.relative = false;
		goal.trajectory.joint_names.push_back("HeadPitch");
		goal.trajectory.joint_names.push_back("HeadYaw");

		goal.trajectory.points.resize(2);
		int step = 0;

		goal.trajectory.points[step].positions.resize(2);
		goal.trajectory.points[step].positions[0] = 0.075124;
		goal.trajectory.points[step].positions[1] = 0.0137641;
		goal.trajectory.points[step].velocities.resize(2);
		for (size_t j = 0; j < goal.trajectory.joint_names.size(); ++j)
		{
			goal.trajectory.points[step].velocities[j] = 0.0;
		}

		goal.trajectory.points[step].time_from_start = ros::Duration(1);
		++step;

		goal.trajectory.points[step].positions.resize(2);
		goal.trajectory.points[step].positions[0] = 0.075124;
		goal.trajectory.points[step].positions[1] = 0.0137641;
		goal.trajectory.points[step].velocities.resize(2);
		for (size_t j = 0; j < goal.trajectory.joint_names.size(); ++j)
		{
			goal.trajectory.points[step].velocities[j] = 0.0;
		}

		goal.trajectory.points[step].time_from_start = ros::Duration(1.84);

		return goal;
    }

    //Second function --> NAO Moving head in yaw and pitch.

    naoqi_msgs::JointTrajectoryGoal detectAngle(vector<double> pitchAndYaw)
    {
    	naoqi_msgs::JointTrajectoryGoal goal;
		goal.relative = false;
		goal.trajectory.joint_names.push_back("HeadPitch");
		goal.trajectory.joint_names.push_back("HeadYaw");

		pitchAndYaw.at(0) = pitchAndYaw.at(0) * M_PI /180;
		pitchAndYaw.at(1) = pitchAndYaw.at(1) * M_PI /180;

		goal.trajectory.points.resize(2);
		int step = 0;
		goal.trajectory.points[step].positions.resize(2);
		goal.trajectory.points[step].positions[0] = pitchAndYaw.at(0);
		goal.trajectory.points[step].positions[1] = pitchAndYaw.at(1);

		goal.trajectory.points[step].velocities.resize(2);
		for (size_t j = 0; j < goal.trajectory.joint_names.size(); ++j)
		{
			goal.trajectory.points[step].velocities[j] = 0.0;
		}

		goal.trajectory.points[step].time_from_start = ros::Duration(1 + 0.5);
		++step;

		goal.trajectory.points[step].positions.resize(2);
		goal.trajectory.points[step].positions[0] = pitchAndYaw.at(0);
		goal.trajectory.points[step].positions[1] = pitchAndYaw.at(1);
		goal.trajectory.points[step].velocities.resize(2);
		for (size_t j = 0; j < goal.trajectory.joint_names.size(); ++j)
		{
			goal.trajectory.points[step].velocities[j] = 0.0;
		}

		goal.trajectory.points[step].time_from_start = ros::Duration(2.50 + 0.5);

		return goal;
    }

