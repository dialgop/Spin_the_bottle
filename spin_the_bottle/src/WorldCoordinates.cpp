/*****************************************************************
 * SPIN THE BOTTLE GAME: By Diego Alejandro Gómez Pardo.		 *
 *																 *
 *  Laboratory in Humanoid Robotics								 *
 *  Humanoid Robots Group										 *
 *  Computer Siences Department 4								 *
 *	Rheinische Friedrich Wilhelms Universität Bonn				 *
 *																 *
 *	WorldCoordinates class: This class contains all of the 		 *
 *	transformations from pixel coordinates to world coordinates	 *
 *	and provides the angle in yaw that NAO needs to turn its head*
 *	and look at the area chosen by the bottle					 *
 *****************************************************************/

#include "../include/spin_the_bottle/WorldCoordinates.h"

/* Measurements in centimeters of the height of both cameras mapped
 * with the coordinates used for the transformation pixel to world
 * coordinates.
 */

double NaoTopCameraHeight = 52.5;
double NaoBottomCameraHeight = 27.5;

/* Ratio of the abstract environment used by NAO in order to use it as
 * intercepting point of the bottle projection.
 */

double circleRadius = 100; //1 meter  = 100 cm

//Constructor class

WorldCoordinates::WorldCoordinates(vector<Point3d> xyangles, Mat& src)
{

}

// This function returns the minimum value of the 2 "y" values. If the
// minimum is negative then the returned point will be (-1,-1)

Point2d WorldCoordinates::getMinimumValue(Point2d x1y1, Point2d x2y2)
{
	cout << "enter in getMinimum value" << endl;

	if(x1y1.y < x2y2.y)
	{
		if(x1y1.y>0)
		{
			return x1y1;
		}
		else
		{
			x1y1.x=-1;
			x1y1.y=-1;
			return x1y1;
		}
	}
	else //if(x2y2.y < x1y1.y)
	{
		if(x2y2.y>0)
		{
			return x2y2;
		}
		else
		{
			x2y2.x=-1;
			x2y2.y=-1;
			return x2y2;
		}
	}
}

vector<double> WorldCoordinates::getWorldCoordinates(vector<Point3d> xyangles, Mat &src)
{
	//Nao moves 55 cm from the center of the board.

		double naoPosWorldInY = 45;

	// Center of the bottle in pixel coordinates is push_back 0. This point
	// contains the x position in .x, y position in .y and angle in .z
	// End of the line of projected bottle is push_back 1. just contains .x and .y

		Point3d center = xyangles.at(0);
		Point3d edge = xyangles.at(1);

	//From OpenCV coordinates to Cartesian coordinates

		Point3d cartesianCenter;
		Point2d cartesianProjection;

		//Convert center of reference from top-left to bottom-left as cartesian coordinates
		cartesianCenter.x = center.x;
		cartesianCenter.y = src.rows - center.y;

		// If the angle is negative it just needs to become positive, otherwise it will
		// substract from 360 degrees.

		if(center.z < 0)
		{
			cartesianCenter.z = -center.z;
		}
		else
		{
			cartesianCenter.z = 360.0 - center.z;
		}

		cartesianProjection.x = edge.x;
		cartesianProjection.y = src.rows - edge.y;

	//From pixel to camera sensor, camera sensor position is pointing to the
	//middle of the image.

		int middleX = src.cols/2;
		int middleY = src.rows/2;

		Point3d cameraCenter;
		Point2d cameraProjection;

		//Camera is pointing to the middle of the image

		cameraCenter.x = cartesianCenter.x - middleX;
		cameraCenter.y = cartesianCenter.y - middleY;
		cameraCenter.z = cartesianCenter.z;

		cameraProjection.x = cartesianProjection.x - middleX;
		cameraProjection.y = cartesianProjection.y - middleY;

	//From camera sensor to NAO measurements.

		//phase 1 from 2d to 3d.
		double angle = cameraCenter.z;
		Point3f naoCenter;
		Point3f naoProjection;

		naoCenter.x = cameraCenter.x;
		naoCenter.y = cameraCenter.y;
		naoCenter.z = 6;

		naoProjection.x = cameraProjection.x;
		naoProjection.y = cameraProjection.y;
		naoProjection.z = 6;

		//Phase 2 from Nao position to image position (convert a mean of 2.5 pixels
		// to 1 cm and homogeneous coordinates to rotate head in case)

		naoCenter.x = naoCenter.x/2.5;
		naoCenter.y = naoCenter.y/2.5;
		naoCenter.z = 2.5;

		naoProjection.x = naoProjection.x/2.5;
		naoProjection.y = naoProjection.y/2.5;
		naoProjection.z = 2.5;

		//From camera sensor to world

		// origin (0,0,0) is at nao feet. Nao feet is always to 55

		double worldCoordCenterX = naoCenter.x;
		double worldCoordCenterY = naoCenter.y + naoPosWorldInY;
		double worldCoordCenterZ = 3;
		double worldCoordProjectX = naoProjection.x;
		double worldCoordProjectY = naoProjection.y + naoPosWorldInY;
		double worldCoordProjectZ = 3;
		double worldCoordAngle = angle;

		vector<double> coordinates;
		coordinates.push_back(worldCoordCenterX);
		coordinates.push_back(worldCoordCenterY);
		coordinates.push_back(worldCoordCenterZ);
		coordinates.push_back(worldCoordProjectX);
		coordinates.push_back(worldCoordProjectY);
		coordinates.push_back(worldCoordProjectZ);
		coordinates.push_back(worldCoordAngle);

		return coordinates;
}

Point2d WorldCoordinates::getPointingCoordinates(vector<double> centerAndEdgeInWorld)
{
	//If returned result is (-1,-1) then the bottle needs to be re-spinned

	Point2d coordinates;

	double a, b, c, x1, x2, y1, y2, finalX, finalY, angle;

	if(centerAndEdgeInWorld.at(6)>85 && centerAndEdgeInWorld.at(6)<95)
		//Exception to delimitations 0: If angle is between 85 and 95.
	{
		x1 = centerAndEdgeInWorld.at(0);
		y1 = sqrt((circleRadius-x1));

		coordinates.x = x1;
		coordinates.y = y1;

		return coordinates;
	}
	else if(!(centerAndEdgeInWorld.at(6)>=205 && centerAndEdgeInWorld.at(6)<335))
		//first delimitation: If angle is between 225° and 315°
	{
		// Using analytic geometry, find the intersection point between the equation formed
		// by the line (center of bottle + angle) and a circumference of 1 meter (100 cm) of
		// distance.

		double angleInRadians = centerAndEdgeInWorld.at(6) * M_PI / 180.0;

		a = 1 + ( tan(angleInRadians)* tan(angleInRadians));
		b = 2 * (tan(angleInRadians)* tan(angleInRadians)) * centerAndEdgeInWorld.at(0);
		c = centerAndEdgeInWorld.at(1) + (tan(angleInRadians)* tan(angleInRadians)) * (centerAndEdgeInWorld.at(0)*centerAndEdgeInWorld.at(0)) - (circleRadius * circleRadius);

		//Now apply Quadratic formula to find the possible x values

		x1 = (-b + sqrt((b*b)-(4*a*c)))/(2*a);

		x2 = (-b - sqrt((b*b)-(4*a*c)))/(2*a);

		//Now find y values replacing in the equation of a line

		y1 = centerAndEdgeInWorld.at(1) + (tan(angleInRadians)*(x1-centerAndEdgeInWorld.at(0)));

		y2 = centerAndEdgeInWorld.at(1) + (tan(angleInRadians)*(x2-centerAndEdgeInWorld.at(0)));

		cout << "wc x1 " << x1 << " wc y1 " << y1 << endl;
		cout << "wc x2 " << x2 << " wc y2 " << y2 << endl;

		// Having the x and y values with the angle, choose the correct one (and also if the correct one applies):

		//Applying the cases where there is a returning point2d and when there is not a returning point2d

		if((centerAndEdgeInWorld.at(6) >= 0 && centerAndEdgeInWorld.at(6) < 85) || (centerAndEdgeInWorld.at(6) >95 && centerAndEdgeInWorld.at(6) <180))
			//Case1: If angle is between 0 and 180° --> 2 possible results...
		{
			if(centerAndEdgeInWorld.at(6)<90)
				//Case1.1: If angle is between 0° and 90°
			{
				if(x1<0)
				{
					coordinates.x = x2;
					coordinates.y = y2;
					return coordinates;
				}
				else
				{
					coordinates.x = x1;
					coordinates.y = y1;
					return coordinates;
				}
			}
			else
				//Case1.2: If angle is between 90° and 180°
			{
				if(x1<0)
				{
					coordinates.x = x1;
					coordinates.y = y1;
					return coordinates;
				}
				else
				{
					coordinates.x = x2;
					coordinates.y = y2;
					return coordinates;
				}
			}
		}
		else
			//Case2: If angles are between 180° and 225° or between 315 and 360°.
		{
			Point2d x1y1, x2y2;

			x1y1.x = x1;
			x1y1.y = y1;

			x2y2.x = x2;
			x2y2.y = y2;

			coordinates = getMinimumValue(x1y1, x2y2);
			cout << "coordinates in x " << coordinates.x << " coordinates in y: " << coordinates.y << endl;
			if(coordinates.y>0)
				//If smallest value is smaller than 0
			{
				return coordinates;
			}
			else
			{
				cout << "Please spin the bottle again (Nao cannot reach that angle in yaw)" << endl;
				return coordinates;
			}
		}
	}
	else
	{
		coordinates.x = -1;
		coordinates.y = -1;
		cout << "Please spin the bottle again (bottle angles between 205° and 335°)" << endl;
		return coordinates;
	}
}

vector <double> WorldCoordinates::getPitchYawHead(vector<double> centerAndEdgeInWorld, Point2d coordinates)
{
	vector <double> pitchYaw;
	double finalX = coordinates.x, finalY = coordinates.y;

	//Calculating new Pitch and Yaw with old procedures

    //Calculating Pitch

	double result = atan2(NaoBottomCameraHeight,circleRadius);

	//cout << "result Pitch" << result * 180 / PI << endl;

	pitchYaw.push_back(result);

	//Calculating Yaw

	cout << "angle" << centerAndEdgeInWorld.at(6) << endl;

	result = atan2 (finalY,finalX);

	cout <<"finalX " << finalX << " finalY " << finalY << " result " << result << endl;

	//Part of dividing the bottle pointing in 5 different areas based on yaw

	if(result >= 0 && result < (36.0*M_PI/180.0))
	{
		result = 18.0*M_PI/180.0;
	}
	else if(result >= (36.0*M_PI/180.0) && result < (72.0*M_PI/180.0))
	{
		result = 54.0*M_PI/180.0;
	}
	else if(result >= (72.0*M_PI/180.0) && result < (108.0*M_PI/180.0))
	{
		result = 90.0*M_PI/180.0;
	}
	else if(result >= (108.0*M_PI/180.0) && result < (144.0*M_PI/180.0))
	{
		result = 126.0*M_PI/180.0;
	}
	else if(result >= (144.0*M_PI/180.0) && result <= (180.0*M_PI/180.0))
	{
		result = 162.0*M_PI/180.0;
	}

	pitchYaw.push_back(result);

	return pitchYaw;
}
