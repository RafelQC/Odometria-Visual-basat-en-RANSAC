#ifndef FEATURE_H_
#define FEATURE_H_

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

#include "Data.h"
#include "Parameters.h"


using namespace cv;

class Feature{
private:
	int m_numMin;
	vector <DMatch> m_good_matches, m_matches;
	vector <Point2f> m_good_features1, m_good_features2;
	vector <int> m_saveGoodCompose;
	double m_fx, m_fy, m_h;
	vector <KeyPoint> m_currentKeypoints;
	vector <KeyPoint> m_prevKeypoints;
	Mat m_currentDescriptor, m_prevDescriptor, m_img, m_cameraMatrix, m_distCoeffs, m_cameraMatrixInv;
public:
	Feature();
	void allFeatures(Mat &img, int method);
	vector <KeyPoint> FastFeatures(Mat img);
	void allDescriptors(int method2);

	vector <DMatch> &bruteforceMatcher();
	vector <DMatch> &FlannMatcher();
	void onlyGoodMatches (vector <DMatch> &match);

	void ordenarFeatures();
	void coordenades();
	void pixelToMeters();
	void configParameters(Parameters *Param);

	void undistortKeyPoints();

	vector <DMatch> &getOnlyGoodFeatures();
	vector <Point2f> &getCenteredFeatures1();
	vector <Point2f> &getCenteredFeatures2();
	void savePrevValues();
	vector <KeyPoint> &getPrevFeature();
	vector <KeyPoint> &getCurrFeature();
};
#endif
