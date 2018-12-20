#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <vector>
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <cstdio>
#include <sys/stat.h>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2\nonfree\features2d.hpp"   //avon es troba SURF detection

using namespace cv;
using namespace std;


class Parameters{
	private:
		
	public:
		Mat CameraMatrix, distCoeffs;
		int numMin, xFrame, selec_3;
		double h;

		int RANSAC_numIteration, RANSAC_numDatapoints, RANSAC_minConsensus, RANSAC_optionRansac;
		double RANSAC_maxError;

		string url;
};

#endif