#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;


class PreGame
{
private:

public:
	PreGame(Mat &src);
	Mat getSaturationChannelValues(Mat src);
	int BottleDetected(Mat &src);
	bool MovementDetected(Mat &src1, Mat &src2);
};
