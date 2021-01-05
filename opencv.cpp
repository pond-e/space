#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;
int main(){
    Mat img = imread("root.jpg", IMREAD_UNCHANGED);
    imshow("IMAGE",img);
    waitKey(10000);
    return 0;
}
