#ifndef DATA_H_
#define DATA_H_

#include <vector>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include "opencv2\nonfree\features2d.hpp"   //avon es troba SURF detection

class Data{
	private:
		double m_x,m_y,m_o;
	public:
		Data() {};
		Data(double x, double y, double o);
		double getX();
		double getY();
		double getO();
		Data compose(Data thePose);
};
#endif