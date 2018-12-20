#include "Feature.h"

Feature::Feature(){
}

void Feature::allFeatures(Mat &img, int method){
	m_matches.clear();
	m_good_matches.clear();
	m_good_features2.clear();
	m_good_features1.clear();
	m_currentKeypoints.clear();
	m_img = img;

	int minHessian = 400;

	SurfFeatureDetector detector( minHessian );   //surf
	SiftFeatureDetector detector2( minHessian );  //sift
	FastFeatureDetector detector3(15);            //fast

	switch (method){
		
		case 1:
		detector.detect(img, m_currentKeypoints);
		break;

		case 2:
		detector2.detect(img, m_currentKeypoints);
		break;

		case 3:
		detector3.detect(img, m_currentKeypoints);
		break;
	};
}

void Feature::allDescriptors(int method2){
	//Mat descriptors;
	SurfDescriptorExtractor extractor;    //descriptor SURF
	SiftDescriptorExtractor extractor2;   //descriptor SIFT
	
	switch(method2){
		case 1:
		extractor.compute( m_img, m_currentKeypoints, m_currentDescriptor);
		break;
		case 2:
		extractor2.compute( m_img, m_currentKeypoints, m_currentDescriptor);
		break;
	};
	
}
	
		//maching fet amb BRUTE FORCE
vector <DMatch> &Feature::bruteforceMatcher(){

	BFMatcher matcher(NORM_L2);
	matcher.match( m_prevDescriptor, m_currentDescriptor, m_matches );
	
	return m_matches;
}
	//matching fet amb FLANN
vector <DMatch> &Feature::FlannMatcher(){
	
	FlannBasedMatcher matcher;
	matcher.match( m_prevDescriptor, m_currentDescriptor, m_matches );
	
	return m_matches;
}
	
	//only good matching! (mirant que les distancies siguin menors que 2*distancia_minima)
void Feature::onlyGoodMatches(vector <DMatch> &match){
	double max_dist = 0; double min_dist = 40000;  //rang de valors inicials que no afecti a les comparacions següents

	if(m_numMin != 0){							 //Condició si volem o no realitzar el llindar, "0" no volem cap llindar
	for( int i = 0; i < match.size(); i++ ){     //Miram quina es la distancia maxima y minima entre tots els keypoints
		double dist = match[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}
		for( int i = 0; i < match.size(); i++ ){            //guardam únicament els valors que compleixen el llindar establert
			if( match[i].distance < m_numMin*min_dist ){
			 m_good_matches.push_back( match[i]);
			}
		}
		printf( "FLANN // Total Matches: %i -->", match.size());
		printf( " Good Matches: %i \n", m_good_matches.size());
}
	if(m_numMin == 0){      //si esta a 0 vol dir que no volem l'opció de fer goodmatch per tant volem tots els matchs
		m_good_matches = match;
	}

	if(m_good_matches.size() == 0){	  //normalment en cas de que sigui la mateixa imatge agafarem tots els keypoints ja que son bons tots
		m_good_matches = match;
	}

}

void Feature::ordenarFeatures(){
	
		long match1, match2;
	for( int i = 0; i < m_good_matches.size(); i++ ){ 
		
		match1 = m_good_matches[i].queryIdx;		//agafam els valors dels numeros de KeyPoint que son parells match	
		match2 = m_good_matches[i].trainIdx;

		m_good_features1.push_back(m_prevKeypoints[match1].pt);   //amb els dos numeros guardam les coordenates dels KeyPoints desitjats
		m_good_features2.push_back(m_currentKeypoints[match2].pt);
	}
}

void Feature::coordenades(){
	//posam un centre de coordenades
	int rows = m_img.rows;    // y
	int cols = m_img.cols;    // x
	
		for( int i = 0; i < m_good_features1.size(); i++ ){
			m_good_features1[i].x = m_good_features1[i].x - (cols/2);
			m_good_features1[i].y = m_good_features1[i].y - (rows/2);
			m_good_features2[i].x = m_good_features2[i].x - (cols/2);
			m_good_features2[i].y = m_good_features2[i].y - (rows/2);
		}

}


vector <Point2f> &Feature::getCenteredFeatures1(){
	return m_good_features1;
}

vector <Point2f> &Feature::getCenteredFeatures2(){
	return m_good_features2;
}

vector <DMatch> &Feature::getOnlyGoodFeatures(){
	return m_good_matches;
}


void Feature::savePrevValues(){
	m_prevKeypoints = m_currentKeypoints;
	m_prevDescriptor = m_currentDescriptor.clone();
}

vector <KeyPoint> &Feature::getPrevFeature(){
	return m_prevKeypoints;
}

vector <KeyPoint> &Feature::getCurrFeature(){
	return m_currentKeypoints;
}

void Feature::configParameters(Parameters *Param){

	
		m_numMin = (*Param).numMin;
		m_cameraMatrix = (*Param).CameraMatrix;
		m_distCoeffs = (*Param).distCoeffs;
		m_h = (*Param).h;


		cv::Mat matrix = cv::Mat(3, 3 ,CV_64F);           //cream una matriu sense el centre de coordenades
		matrix.at<double>(0,0) = m_cameraMatrix.at<double>(0,0);         //  |fx  0  0|
		matrix.at<double>(1,1) = m_cameraMatrix.at<double>(1,1);         //  |0  fy  0|
		matrix.at<double>(2,2) = 1;                                      //  |0   0  1|
		matrix.at<double>(0,1) = 0; matrix.at<double>(0,2) = 0;
		matrix.at<double>(1,0) = 0; matrix.at<double>(1,2) = 0;
		matrix.at<double>(2,0) = 0; matrix.at<double>(2,1) = 0;

		m_cameraMatrixInv = cv::Mat(3, 3 ,CV_64F);
		m_cameraMatrixInv = matrix.inv();                   //invertim la matriu per despres aplicar l'ecuació per passar a metros
		m_fx = m_cameraMatrixInv.at<double>(0,0);			   //guardam fx i fy per despres fer l'operació per Pixel -> Metros
		m_fy = m_cameraMatrixInv.at<double>(1,1);
}


void Feature::pixelToMeters(){

	for( int i = 0; i < m_good_features1.size(); i++ ){
			m_good_features1[i].x = m_good_features1[i].x*m_h*m_fx;
			m_good_features1[i].y = m_good_features1[i].y*m_h*m_fy;
			m_good_features2[i].x = m_good_features2[i].x*m_h*m_fx;
			m_good_features2[i].y = m_good_features2[i].y*m_h*m_fy;
		}
	
}

void Feature::undistortKeyPoints(){
	//Pasar de KeyPoint a Matriu de vectors 2f
    cv::Mat src = cv::Mat(1,m_currentKeypoints.size(),CV_32FC2); 
    int i = 0;
    for (std::vector<cv::KeyPoint>::iterator it = m_currentKeypoints.begin(); it != m_currentKeypoints.end(); it++){
        src.at<cv::Vec2f>(0,i)[0] = (*it).pt.x;
        src.at<cv::Vec2f>(0,i)[1] = (*it).pt.y;
        i++;
    }

	cv::Mat norm = cv::Mat(1,m_currentKeypoints.size(),CV_32FC2);

	//desdistorsionam els pixels
    cv::undistortPoints(src, norm, m_cameraMatrix, m_distCoeffs);

	//tornam a guardar els Keypoints desdistorsionats a un vector tipus KeyPoint, desnormalitzant els punts empleant fx, fy, cx, cy de CameraMatrix
	//desnormalitzam perque necessitarem les coordenades en píxels, ja que posteriorment imprimim els matching i es necessari emplear píxels com a mesura
	i = 0;
    for (std::vector<cv::KeyPoint>::iterator it = m_currentKeypoints.begin(); it != m_currentKeypoints.end(); it++){
		(*it).pt.x = (norm.at<cv::Vec2f>(0,i)[0])*(m_cameraMatrix.at<double>(0,0))+(m_cameraMatrix.at<double>(0,2));
		(*it).pt.y =  (norm.at<cv::Vec2f>(0,i)[1])*(m_cameraMatrix.at<double>(1,1))+(m_cameraMatrix.at<double>(1,2));
        i++;
    }
}