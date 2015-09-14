/*****************************************************************
 * SPIN THE BOTTLE GAME: By Diego Alejandro Gómez Pardo.		 *
 *																 *
 *  Laboratory in Humanoid Robotics								 *
 *  Humanoid Robots Group										 *
 *  Computer Siences Department 4								 *
 *	Rheinische Friedrich Wilhelms Universität Bonn				 *
 *																 *
 *	Subscriber node: This is a ROS node which contains the NAO 	 *
 *	face detection package. It was not used in the program		 *
 *****************************************************************/

#include <nao_interaction_msgs/FaceDetected.h>
#include <ros/ros.h>

class FaceDetection
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	nao_interaction_msgs::FaceDetected face;
	bool face_detection_enable;
public:
	FaceDetection()
{
		sub = nh.subscribe("/nao_vision/faces_detected",1,&FaceDetection::callback,this);
		face_detection_enable = true;
}
	nao_interaction_msgs::FaceDetected  perform()
	{
		ROS_INFO_STREAM("in FaceDetected Perform");
		nao_interaction_msgs::FaceDetected facedet;
		ros::Rate r(0.5);
		while (face_detection_enable)
		{
			ROS_INFO_STREAM("spinning once");
		ros::spinOnce();
		r.sleep();
		}
			facedet = face;

		return face;
	}
	void callback(const nao_interaction_msgs::FaceDetected& facemsgs)
	{
		ROS_INFO_STREAM("in FaceDetected callback");

		this->face = facemsgs;
		ROS_INFO_STREAM("Header " << facemsgs.header.frame_id.c_str());
		ROS_INFO_STREAM("Alpha " <<facemsgs.shape_alpha.data);
		ROS_INFO_STREAM("Beta " << facemsgs.shape_beta.data);
		face_detection_enable = false;
		/*

		if (facemsgs.face_id.data)
		{
			this->face = facemsgs;
			ROS_INFO_STREAM("Face id found :"<<facemsgs.face_id.data);
			face_detection_enable = false;
		}
		if (facemsgs.right_eye_eyeCenter_x.data)
		{
			this->face = facemsgs;
			ROS_INFO_STREAM("right_eye_eyeCenter_x :"<<facemsgs.right_eye_eyeCenter_x.data);
			face_detection_enable = false;
		}
		if (facemsgs.right_eye_eyeCenter_y.data)
		{
			this->face = facemsgs;
			ROS_INFO_STREAM("right_eye_eyeCenter_y :"<<facemsgs.right_eye_eyeCenter_y.data);
			face_detection_enable = false;
		}
		if (facemsgs.left_eye_eyeCenter_x.data)
		{
			this->face = facemsgs;
			ROS_INFO_STREAM("left_eye_eyeCenter_x :"<<facemsgs.left_eye_eyeCenter_x.data);
			face_detection_enable = false;
		}
		if (facemsgs.left_eye_eyeCenter_y.data)
		{
			this->face = facemsgs;
			ROS_INFO_STREAM("left_eye_eyeCenter_y :"<<facemsgs.left_eye_eyeCenter_y.data);
			face_detection_enable = false;
		}*/
	}
};


