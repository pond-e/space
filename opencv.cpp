#include "opencv/cv.h"
#include "opencv/highgui.h"
using namespace cv;
int main(){
    Mat img = imread("root.jpg", IMREAD_UNCHANGED);
    imshow("IMAGE",img);
    waitKey(10000);
    return 0;
}
