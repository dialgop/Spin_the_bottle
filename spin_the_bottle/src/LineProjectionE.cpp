/*****************************************************************
 * SPIN THE BOTTLE GAME: By Diego Alejandro Gómez Pardo.		 *
 *																 *
 *  Laboratory in Humanoid Robotics								 *
 *  Humanoid Robots Group										 *
 *  Computer Siences Department 4								 *
 *	Rheinische Friedrich Wilhelms Universität Bonn				 *
 *																 *
 *	LineProjectionE class: This class contains all the methods   *
 *	for the orientation and angle projection of the bottle       *
 *****************************************************************/

#include "../include/spin_the_bottle/LineProjectionE.h"

double thresholdValue = 70; //Change here --> Before it was 0

/* Borders are removed from the visual field in order to avoid contours
 * not belonging to the bottle (like the ones from the board)
 */

short int borderPixelsTB = 30;
short int borderPixelsLR = 20;

//LineProjection constructor

LineProjectionE::LineProjectionE(Mat &matrix) 
{

}

//Function to get the channel of green values in a 1 channel image.

Mat LineProjectionE::getSaturationChannelValues(Mat &matrixReaded)
{
	Mat imageHSV;

	//Converts image into HSV

	cvtColor(matrixReaded,imageHSV,CV_BGR2HSV);

	//To divide the image into its 3 channels (We will use Saturation)

	Mat channels[3];

	split(imageHSV,channels);

	/* To invert the image in order to make threshold later
	 * (Threshold cuts higher values) and important values are
	 * white (255) here
	 */

	//For S Channel

	Mat invertedImageS;

	bitwise_not ( channels[1], invertedImageS );

	//Removes the edges of the platform

	for (int i = 0; i < invertedImageS.rows ; i++)
	{
		for (int j = 0; j < invertedImageS.cols ; j++)
		{
			if(!((i>invertedImageS.rows/4) && (j>invertedImageS.cols/6 && j<invertedImageS.cols*5/6)))
			{
				invertedImageS.at<uchar>(i, j) = 255;
			}
		}
	}

	//Applies threshold taking into account dark pixels (values from 0 to 70) in S and V channel

	Mat thresholdImgS;

	threshold(invertedImageS, thresholdImgS, thresholdValue,255,THRESH_BINARY_INV);

	Mat thresholdImgV;

	threshold(channels[2], thresholdImgV, thresholdValue ,255,THRESH_BINARY_INV);

	Mat output = thresholdImgS.mul(thresholdImgV);

	return output;
}

vector<int> LineProjectionE::FindPointingArea(Mat matrixReaded)
{
//Creates a 5 pos vector which will contain the values like this |(0)region|(1)top|(2)bottom|(3)left|(4)right|

	vector<int> edgeAndBoundaries;

// Creates a vector result which will store the threshold values of the intersected S
// and V channels of the source image.

	Mat binaryImage = getSaturationChannelValues(matrixReaded);

//Creates an auxiliar matrix which will contain the same threshold as explained before.

	Mat binaux = getSaturationChannelValues(matrixReaded);

// Find contours of the threshold (after applying threshold, the image loses quality) --> aux
// contours is a vector which contains a vector of points (with pixels) which represent a contour.
// Since there are several contours, then it's necessary a vector of vectors

	vector<vector<Point> > contours;

// Despite hierarchy is not used here, it establishes the mode of the contour retrieval
// algorithm. Since external is used, then just the most external contours are passed into
// the hierarchy (not the ones contained inside the contours).

	vector<Vec4i> hierarchy;

	findContours( binaryImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

//Merge all contours into a vector

	std::vector<cv::Point> merged_contour_points;

	for (size_t i = 0; i < contours.size(); i++) {
		for (size_t j = 0; j < contours[i].size(); j++)
		{
			merged_contour_points.push_back(contours[i][j]);
		}
	}

// Gets the hull points which represent the extreme salient parts of the bottle

	std::vector<cv::Point> hull;
	cv::convexHull(cv::Mat(merged_contour_points),hull, false);
	cv::Mat hull_points(hull);

//Fills the Bottle inside, this is the one which will be used in order to get orientation

	fillConvexPoly(binaux,hull_points,Scalar(255));

// Gets top, bottom, left and right positions of the bottle.
// BoundingRect function creates a square able to contain all of the points (in this case
// hull points) that are passed as parameter.

	cv::Rect rectangle_bounded = boundingRect(cv::Mat(hull_points));

// Based on the top, bottom, left and right positions of the bottle,
// calculates the area less populated of pixels (this area should be
// the place where the bottle points). this is done counting pixels
// between the segment conformed by Top,Bottom,Left,Right.

// There is a trick about the results of the bottle. If it's 1 or 3 then
// x values are larger than y values (and 1 means pointing from 90 to 270°
// in human angles convention and 3 means poiting from 0 to 90 and from 270
// to 360). If it is 2 or 4 then y values are larger than x ones (and 2 means
// pointing from 0 to 180° in human angle convention and 4 means pointing from
// 180 to 360°) This will be used for the angle obtained from a function called
// fitEllipse in the method getPointingLine.

	rectangle( binaux, rectangle_bounded.tl(), rectangle_bounded.br(), Scalar(100), 1, 8, 0 );

	int top = rectangle_bounded.tl().y , bottom = rectangle_bounded.br().y, left = rectangle_bounded.tl().x, right = rectangle_bounded.br().x;
	int pointer=0;

	if((bottom-top)>=(right-left))
	{
		int pix_up = 0, pix_down = 0;

		for (int i = top; i < bottom;i++)
		{
			for (int j = left; j < right; j++)
			{
				if((int)binaux.at<uchar>(i, j) == 255)
				{
					if(i<(top + ((bottom - top)/2)))
					{
						pix_up++;
					}
					else
					{
						pix_down++;
					}
				}
			}
		}

		if(pix_up < pix_down)
		{
			pointer = 2;	//bottle points towards zero in y
		}
		else
		{
			pointer = 4;	//bottle points towards image.rows in y
		}

		line( binaux, Point(left, top+((bottom-top)/2)), Point(right, top+((bottom-top)/2)), Scalar(128), 1, 8 );
	}
	else
	{
		int pix_left = 0, pix_right = 0;

		for (int i = top; i < bottom ;i++)
		{
			for (int j = left; j < right; j++)
			{
				if((int)binaux.at<uchar>(i, j)== 255)
				{
					if(j < (left+((right-left)/2)))
					{
						pix_left++;
					}
					else
					{
						pix_right++;
					}
				}
			}
		}

		if(pix_left < pix_right)
		{
			pointer = 1;	//bottle points towards zero in x
		}
		else
		{

			pointer = 3;	//bottle points towards image.cols in x
		}

		line( binaux, Point(left+((right-left)/2), top), Point(left+((right-left)/2), bottom), Scalar(128), 1, 8 );
	}

//Puts in the vector edgeAndBoundaries the values of the pointing place and the top, bottom , left and right and returns it.

	edgeAndBoundaries.push_back(pointer);
	edgeAndBoundaries.push_back(top);
	edgeAndBoundaries.push_back(bottom);
	edgeAndBoundaries.push_back(left);
	edgeAndBoundaries.push_back(right);

	return edgeAndBoundaries;
}

vector <Point3d> LineProjectionE::drawPointingLine(Mat &src, vector<int> edge_bound)
{
//Creates a point3d called xyangle which will contain the x,y value of center and also
//the angle created by with the function fitEllipse

	Point3d xyangle;

// Creates a vector of vectors called Hierarchy which tells if a specified point has neighbors
// Also creates a vector of vector of points which will contain the contours created after
// using the imgBinary with the function findContours (same as previous explained function)

	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;

// Creates a vector result which will store the threshold values of the intersected S and
// V channels of the source image.

	Mat binaryImage = getSaturationChannelValues(src);

	Mat threshAux = getSaturationChannelValues(src);

// Find contours and merge them again with convex_hull.

	findContours( binaryImage, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

// merge all contours into one vector

	std::vector<cv::Point> merged_contour_points;

	for (size_t i = 0; i < contours.size(); i++) {
		for (size_t j = 0; j < contours[i].size(); j++)
		{
			merged_contour_points.push_back(contours[i][j]);
		}
	}

	std::vector<cv::Point> hull;
	cv::convexHull(cv::Mat(merged_contour_points),hull, false);
	cv::Mat hull_points(hull);

	vector <Point3d> points2D;

	try
	{

	cv::RotatedRect minEllipse = fitEllipse(hull_points);

// Creates the points center and angle which will be passed later to the xyangle, p1 is the
// auxiliar line which is the second point where the line will be drawn

	Point center = minEllipse.center;
	float angle = minEllipse.angle - 90;

// Now the findPointingEdge method will be used in order to modify the angle (in case
// orientation region is not calibrated

	if(edge_bound.at(0)==1 &&((angle>-90 && angle<-0) || (angle >0 && angle<90)))
	{
		angle = angle + 180;
	}
	else if(edge_bound.at(0) == 2 && (angle >0 && angle<180))
	{
		angle = angle + 180;
	}
	else if(edge_bound.at(0) == 3 && (angle >90 && angle<270))
	{
		angle = angle + 180;
	}
	else if(edge_bound.at(0) == 4 && ((angle>-90 && angle<-0)||(angle>180 && angle<270)))
	{
		angle = angle + 180;
	}

	xyangle.x = center.x;
	xyangle.y = center.y;
	xyangle.z = angle;

	Point endpoint;

	endpoint.x = (center.x + 100 * cos(angle*M_PI/180));
	endpoint.y = (center.y + 100 * sin(angle*M_PI/180)) ;

	Point3d endProjection;

	endProjection.x = endpoint.x;
	endProjection.y = endpoint.y;

	//Main line

	ellipse( src, minEllipse, Scalar(180), 2, 8 );

	line(src,center, endpoint,Scalar(200),4);

	//Hook 1

	center.x = (int) (endpoint.x - 12 * cos(angle*M_PI/180 + CV_PI / 4));
	center.y = (int) (endpoint.y - 12 * sin(angle*M_PI/180 + CV_PI / 4));

	line(src,center, endpoint,Scalar(200),4);

	//Hook 2

	center.x = (int) (endpoint.x - 12 * cos(angle*M_PI/180 - CV_PI / 4));
	center.y = (int) (endpoint.y - 12 * sin(angle*M_PI/180 - CV_PI / 4));

	line(src,center, endpoint,Scalar(200),4);

	points2D.push_back(xyangle);
	points2D.push_back(endProjection);
	}
	catch(exception e)
	{
		cout << "It was not possible to find the endpoints" << endl;
		points2D.push_back(xyangle);
		points2D.push_back(Point3d (0,0,0));
	}

	return points2D;
}
