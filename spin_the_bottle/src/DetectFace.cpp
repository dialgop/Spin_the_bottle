/*****************************************************************
 * SPIN THE BOTTLE GAME: By Diego Alejandro Gómez Pardo.		 *
 *																 *
 *  Laboratory in Humanoid Robotics								 *
 *  Humanoid Robots Group										 *
 *  Computer Siences Department 4								 *
 *	Rheinische Friedrich Wilhelms Universität Bonn				 *
 *																 *
 *	DetectFace class: This class, based on OpenCV contains the   *
 *	approach of HaarCascades in order to recognize faces. To be	 *
 *	able to accomplish with this task, it is neccesary to count	 *
 *	With the training set provided by the xml file 				 *
 *	lbpcascade_frontalface.xml         							 *
 *****************************************************************/

#include "../include/spin_the_bottle/DetectFace.h"
#include <ros/package.h>
string face_cascade_name = "lbpcascade_frontalface.xml";
CascadeClassifier face_cascade;

DetectFace::DetectFace(Mat &matrix)
{

}

bool DetectFace::detectAndDisplay(Mat frame)
{
	//Loads the xml file

	bool isFace = false;

	if (!face_cascade.load(ros::package::getPath("bottle_recognition") + "/src/" + face_cascade_name))
	{
		cout << "--(!)Error loading the xml file\n" << endl;
		return false;
	}

	std::vector<Rect> faces;
	Mat frame_gray;

	//Changes image from color to Grayscale and normalizes the grayscale image
	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	/*  In order to avoid detecting faces of people in other regions, 1/4 of both
	*	lateral sides of the screen are discarded (they are painted black)
	*/

	for(size_t i = 0; i < frame.rows; i++)
	{
		for(size_t j = 0; j < frame.cols; j++)
		{
			if((j < frame.cols/4 || j >3*frame.cols/4))
			{
				frame_gray.at<uchar>(i, j) = 0;
			}
		}
	}

	// Opencv Method to detect Faces.

	face_cascade.detectMultiScale(frame_gray, faces, 1.1, 3, 0 | CASCADE_SCALE_IMAGE, Size(40, 40));

	if(faces.size()>0)
	{
		Point pt1(faces[0].x, faces[0].y); // Display detected faces on main window
		Point pt2((faces[0].x + faces[0].height), (faces[0].y + faces[0].width));
		rectangle(frame, pt1, pt2, Scalar(0, 255, 0), 2, 8, 0);
		isFace = true;
	}
	return isFace;
}
