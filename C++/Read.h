#ifndef READ_H_
#define READ_H_

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/calib3d/calib3d.hpp>

#include <sstream>  // string to number conversion
#include <iomanip>  // for controlling float print precision
#include <cstdio>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include "opencv2\nonfree\features2d.hpp"   //avon es troba SURF detection

#include "Parameters.h"


using namespace cv;

class Read{
private:
	Mat m_img1, m_img2, m_next_img;
	char m_url [100];

public:
	Mat &ObrirMatriu1(string url1);
	Mat &ObrirMatriu2(string url2);
	Mat &getMatrix1();
	Mat &getMatrix2();
	Mat &nextImg(int counter);
	bool moreFrames(int counter);
	void iniread(string url);
};

#endif