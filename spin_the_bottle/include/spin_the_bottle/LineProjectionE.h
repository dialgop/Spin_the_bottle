#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

const double pi = 3.1415926535897;

class LineProjectionE
{
private:
	Mat getSaturationChannelValues(Mat &Matrix);

public:
	LineProjectionE(Mat &matrix);
	vector<Point3d> drawPointingLine(Mat &src, vector<int> edge_bound);
	vector<int> FindPointingArea(Mat matrixReaded);
};
