#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <iostream>
using namespace cv;
using namespace std;
int main(){
    Mat img = imread("root.jpg", IMREAD_UNCHANGED);
    Mat gray_img;
    cvtColor(img, gray_img, CV_BGR2GRAY);
    Mat bin_img;
    threshold(gray_img, bin_img, 0, 255, THRESH_BINARY|THRESH_OTSU);
    bin_img = ~bin_img;
    vector<vector<Point> > contours;
    findContours(bin_img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    double contour_len=0;
    for(int i = 0; i < contours.size(); ++i) {
        contour_len += arcLength(contours.at(i),1);
    }
    cout<<"estimated root length = "<<contour_len/2<<endl;
    imshow("IMAGE",bin_img); //変更
    waitKey(10000);
    return 0;
}
