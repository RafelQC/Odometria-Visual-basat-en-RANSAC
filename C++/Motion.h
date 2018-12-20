#ifndef MOTION_H_
#define MOTION_H_

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

#include "Data.h"
#include "Parameters.h"


using namespace cv;
using namespace std;

class Motion{	
	private:

		Mat m_img;
		bool m_goodProcess;
		vector <Point2f> m_features1, m_features2, m_maybeInliers, m_maybeInliersPrev;
		int m_RANSAC_numIterations, m_RANSAC_numDataPoints, m_RANSAC_minConsensus, m_RANSAC_minConsensusPercent, m_optionRansac;
		int m_selec3, m_xFrame, m_contador;
		double m_RANSAC_maxError;
		double m_Xprev, m_Yprev, m_Oprev;
		Data m_themotion;
		vector <double> m_odoX, m_odoY, m_odoO, m_composeX, m_composeY, m_composeO;
		string m_url;
		char m_fName[100];
		int m_numMin;
	public:
		Motion();
		void configureRANSAC(Parameters *Param);
		void inimotion (vector <Point2f> &features1, vector <Point2f> &features2, int method);
		bool goodRANSAC();
		Data getMotion();
		vector <Point2f> &getMaybeInliers();
		vector <Point2f> &getMaybeInliersPrev();
		void saveOdoCurrent(double X, double Y, double O);
		void saveOdoPrev();
		void writeCompose(double Time);
		void compose();
		void imprimirRANSAC(Mat img1, Mat img2);

	private:
		Data executeMinSquares2(vector <Point2f> &features1, vector <Point2f> &features2);
		Data executeRANSAC(vector <Point2f> &features1, vector <Point2f> &features2);
		double computeError(vector <Point2f> &features1, vector <Point2f> &features2, Data theMotion);
		void prepareUrlData();
};
#endif
