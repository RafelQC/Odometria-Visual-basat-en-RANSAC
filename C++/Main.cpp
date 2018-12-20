#define VISUAL_DEBUG

#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <cstdio>
#include <sys/stat.h>
#include <windows.h>
#include <ctime>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/calib3d/calib3d.hpp>
#include <sys/stat.h>
#include <highgui.h>
#include "opencv2\nonfree\features2d.hpp"   //on es troba SIFT detection

#include "Read.h"
#include "Feature.h"
#include "Motion.h"
#include "Data.h"
#include "Parameters.h"

using namespace cv;


int main(int argc, char *argv[]){
	
	Mat img_1, img_2, img_matches, img_matches_RANSAC;
	double Time = 0;

	Read Rread;
	Feature Ffeature;
	Motion Mmotion;
	Data Ddata;
	Parameters Param;

	/////Constants - lectura de l'arxiu d'entrada/////
	FileStorage fs("Data/Parameters2.xml", FileStorage::READ); //lectura d'arxiu d'entrada per a la configuració del programa

	Param.url = (string)fs["url"];
	Param.xFrame = (int)fs["xFrame"];
	Param.selec_3= (int)fs["selec3"];
	
	fs["Camera_Matrix"] >> Param.CameraMatrix;        //paràmetres de la càmera
	fs["Distortion_Coefficients"] >> Param.distCoeffs;
	Param.h = (double)fs["h"];
	Param.numMin = (int)fs["numMin"];

	Param.RANSAC_numIteration = (int)fs["numIterations"];  //paràmetres RANSAC
	Param.RANSAC_numDatapoints = (int)fs["numDataPoints"];
	Param.RANSAC_maxError = (double)fs["maxError"];
	Param.RANSAC_minConsensus = (int)fs["minConsensus"];
	Param.RANSAC_optionRansac = (int)fs["optionRansac"];
	

	int selec_1 = (int)fs["selec1"];   //elecció SURF, SIFT o FAST (features & descriptors)
	int selec_2 = (int)fs["selec2"];   //a priori sempre SIFT
		

			/////Save parameters/////
	Rread.iniread(Param.url);
	Mmotion.configureRANSAC(&Param);  //configuració inicial de RANSAC iteracions, nombre de samples, minMatchConsens %, error (llegint fitxer a "data/ConfigRANSAC") nº de ransac al llegir error
	Ffeature.configParameters(&Param);

	img_1 = Rread.nextImg(1);  //lectura de la primera imatge
	Ffeature.allFeatures(img_1,selec_1);       //obtenim els keypoints               Combinacions SIFT -> SIFT, SURF -> SURF, FAST -> SURF
	Ffeature.undistortKeyPoints();      //desdistorsionam el KeyPoints
	Ffeature.allDescriptors(selec_2);     //obtenim els descriptors
	Ffeature.savePrevValues();         //guardar valors anteriors com a imatge anterior

	int counter = 1 + Param.xFrame;
	while(Rread.moreFrames(counter)){
		clock_t c_start = std::clock();
		printf(" -- Frame %i -- \n",counter);

		img_2 = Rread.nextImg(counter);
		Ffeature.allFeatures(img_2, selec_1);	  // 1 = SURF, 2 = SIFT, 3 = FAST
		Ffeature.undistortKeyPoints();			 //desdistorsionam el KeyPoints
		Ffeature.allDescriptors(selec_2);	     // 1 = surf, 2 = SIFT

		Ffeature.onlyGoodMatches(Ffeature.FlannMatcher());   //feim matching i elegim els matches bons amb un mínim

		// Dibuixam els matches
		#ifdef VISUAL_DEBUG
		drawMatches(img_1, Ffeature.getPrevFeature(), img_2, Ffeature.getCurrFeature(), Ffeature.getOnlyGoodFeatures(), img_matches,  Scalar(30, 50, 350), Scalar(30, 50, 350), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
		imshow("Matches", img_matches);
		#endif

		//guardam els features bons i configuram coordenades
		Ffeature.ordenarFeatures();  //guardam els matches bons
		Ffeature.coordenades();    //posam un centre de coordenades
		Ffeature.pixelToMeters();   //pasam els pixels a metros amb un centre de coordenades canviat abans


		//iniciam MOTION
		Mmotion.inimotion(Ffeature.getCenteredFeatures1(), Ffeature.getCenteredFeatures2(),Param.selec_3);     //pasam les dades de Features amb coordenades centrades
																							   //opció 1 = Least Squares, opció 2: RANSAC
			if(Mmotion.goodRANSAC()){
				Ddata = Mmotion.getMotion();
				Mmotion.saveOdoCurrent(Ddata.getX(),-Ddata.getY(), -Ddata.getO());

				// Dibuixam els matches RANSAC
				#ifdef VISUAL_DEBUG
				//Mmotion.imprimirRANSAC(img_1, img_2);
				#endif
			}

		Ffeature.savePrevValues();    //guardar valors anteriors
		img_1 = img_2.clone();
		counter += Param.xFrame;    //pasam el contador al següent frame, Xframe si volem de 1 en 1, o de X en X

		clock_t c_end = std::clock();
		Time = Time + 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;   //guardam els increments de temps a Time per a sebre el temps total
		std::cout << "CPU time used: "
              <<1000.0 * (c_end-c_start) / CLOCKS_PER_SEC
              << " ms\n \n";
		cvWaitKey();
	}
	//guardam al fitxer l'odometria
	Mmotion.compose();
	Mmotion.writeCompose(Time);

		//senyal acustica de que el programa ha finalitzat
		int i;
		char Pitido = 7; 
		for(i=0; i<500;i++){
		cout << Pitido << endl; 

		}
    return 0;
}