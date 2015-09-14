#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

class DetectFace
{
private:

public:
	DetectFace(Mat &matrix);
	bool detectAndDisplay(Mat matrix);
};
