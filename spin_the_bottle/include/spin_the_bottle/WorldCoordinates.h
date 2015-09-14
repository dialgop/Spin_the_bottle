#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <math.h>

#define PI 3.14159265
using namespace cv;
using namespace std;

class WorldCoordinates
{
private:
	Point2d getMinimumValue(Point2d x1y1, Point2d x2y2);
public:
	WorldCoordinates(vector<Point3d> xyangles, Mat &src);
	vector <double> getWorldCoordinates(vector<Point3d> xyangles, Mat &src);
	Point2d getPointingCoordinates(vector<double> centerAndEdgeInWorld);
	vector <double> getPitchYawHead(vector<double> centerAndEdgeInWorld, Point2d coordinates);
};
