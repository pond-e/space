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
    Mat bin_img_copy=bin_img.clone(); //追加
    Mat element = Mat::ones(9,9,CV_8UC1); //追加
    vector<vector<Point> > contours;
    findContours(bin_img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    double contour_len=0;
    for(int i = 0; i < contours.size(); ++i) {
        contour_len += arcLength(contours.at(i),1);
    }
    cout<<"estimated root length = "<<contour_len/2<<endl;
    //追加ここから
    erode(bin_img_copy, bin_img_copy, element, Point(-1,-1), 1);
    imshow("IMAGE_main_root",bin_img_copy);
    vector<vector<Point> > contours_main;
    findContours(bin_img_copy, contours_main, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    double contour_len_main=0;
    for(int i = 0; i < contours_main.size(); ++i) {
        contour_len_main += arcLength(contours_main.at(i),0);
    }
    cout<<"estimated root length (main) = "<<contour_len_main/2<<endl;
    cout<<"estimated root length (lateral) = "<<(contour_len-contour_len_main)/2<<endl;
    //追加ここまで
    imshow("IMAGE",bin_img); //変更
    waitKey(10000);
    return 0;
}
