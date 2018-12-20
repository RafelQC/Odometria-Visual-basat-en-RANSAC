#include "Motion.h"


Motion::Motion(){
}

void Motion::inimotion(vector <Point2f> &features1, vector <Point2f> &features2, int method){
	m_features1 = features1;
	m_features2 = features2;
	m_goodProcess = false;   //variable RANSAC ha trobat solució
	
	switch (method){
		case 1: //MinSquares
			m_goodProcess = true;   //sempre trobarem una solució per tant posam la variable a true
			m_themotion = executeMinSquares2(features1, features2);
		break;

		case 2: //RANSAC
			
				printf("Porcentaje:%d //",m_RANSAC_minConsensus);
				m_RANSAC_minConsensusPercent = (features1.size()*m_RANSAC_minConsensus/100);
				printf(" MatchesTot:%d --> NumeroMin:%d \n",features1.size(),m_RANSAC_minConsensusPercent);
				if(features1.size()>6){      //només usam RANSAC si tenim un minim de features
					m_themotion = executeRANSAC(features1, features2);
				}
		break;
	}
}


Data Motion::executeMinSquares2(vector <Point2f> &features1, vector <Point2f> &features2){
	double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0, xx = 0.0, yy = 0.0, xy = 0.0, yx = 0.0;

	for (unsigned int i=0; i<features1.size(); ++i)
	{
		Point2f p1=features2[i], p2=features1[i];

		x1 += p1.x;
		x2 += p2.x;
		y1 += p1.y;
		y2 += p2.y;
		xx += p1.x*p2.x;
		yy += p1.y*p2.y;
		xy += p1.x*p2.y;
		yx += p1.y*p2.x;
	}

	double N = (double)features1.size();

	double Sxx = xx - x1*x2/N; // calculate S
	double Syy = yy - y1*y2/N;
	double Sxy = xy - x1*y2/N;
	double Syx = yx - y1*x2/N;

	double xm1 = x1/N; // calculate means
	double xm2 = x2/N;
	double ym1 = y1/N;
	double ym2 = y2/N;

	double yaw = atan2(Sxy-Syx, Sxx+Syy);
	double x = xm2 - (xm1*cos(yaw) - ym1*sin(yaw));
	double y = ym2 - (xm1*sin(yaw) + ym1*cos(yaw));

	return Data (x,y,yaw);

}

Data Motion::getMotion(){
	return m_themotion;
}

void Motion::configureRANSAC(Parameters *Param){   //configuarió de RANSAC guardada als fitxers

	m_RANSAC_numIterations = (*Param).RANSAC_numIteration;
	m_RANSAC_numDataPoints = (*Param).RANSAC_numDatapoints;
	m_RANSAC_maxError = (*Param).RANSAC_maxError;
	m_RANSAC_minConsensus = (*Param).RANSAC_minConsensus;
	m_optionRansac = (*Param).RANSAC_optionRansac;

	m_url = (*Param).url;
	m_selec3 = (*Param).selec_3;
	m_xFrame = (*Param).xFrame;
	m_numMin = (*Param).numMin;

	double m_Xprev = 0, m_Yprev= 0, m_Oprev= 0;
	m_contador = 0; //contador per a imprimir OdoPrev
}




Data Motion::executeRANSAC(vector <Point2f> &features1, vector <Point2f> &features2){

	int i, j, theIndex, numFeatures=features1.size();
	double theDistance, x1, y1, x2, y2, thisError, bestError=99999;
	vector <Point2f> maybeInliers;       // vectors on guardarem els
	vector <Point2f> maybeInliersPrev;   //samples aleatoris de matchs
	vector <bool> featureSelected;
	Data maybeModel, thisModel, bestModel;
	Point2f curPoint, prevPoint;
	bool firstTime=true;
	RNG rng( 0xFFFFFFFF );

	// RANSAC bucle principal
	for (i=0;i<m_RANSAC_numIterations;i++) {
		// Seleccionam features aleatoris
		maybeInliers.clear();
		maybeInliersPrev.clear();
		featureSelected.assign(numFeatures,false);
		for (j=0;j<m_RANSAC_numDataPoints;j++){
			theIndex=rng.uniform(0,numFeatures);
			maybeInliers.push_back(features2[theIndex]);      //guardam els valors dels matchs aleatoris
			maybeInliersPrev.push_back(features1[theIndex]);  //als dos nous vectors
			featureSelected[theIndex]=true;
		}
		// Estimam el model amb els parametres de maybeInliers i maybeInliersPrev
		maybeModel=executeMinSquares2(maybeInliersPrev, maybeInliers);
		// Per tots els punts que no estan a maybeInliers
		for (j=0;j<numFeatures;j++) {
			if (!featureSelected[j]) {
				curPoint=features2[j];
				prevPoint=features1[j];
				Data posePoint(curPoint.x, curPoint.y, 0);
				// Movem els punts segons el model
				posePoint=maybeModel.compose(posePoint);
				x1=prevPoint.x;
				y1=prevPoint.y;
				x2=posePoint.getX();
				y2=posePoint.getY();
				x1=(x1-x2)*(x1-x2);
				y1=(y1-y2)*(y1-y2);
				// Computam l'error
				theDistance=sqrt(x1+y1);
				if (theDistance<=m_RANSAC_maxError) {
					maybeInliers.push_back(curPoint);
					maybeInliersPrev.push_back(prevPoint);
				}
			}
		}
		// Si el nombre de features correctes es major que el llindar
		if (maybeInliers.size()>m_RANSAC_minConsensusPercent) {
			m_goodProcess = true;
			thisModel=executeMinSquares2(maybeInliersPrev, maybeInliers);
			thisError=computeError(maybeInliers, maybeInliersPrev, thisModel);
			if (firstTime || thisError<bestError) {
				firstTime=false;
				bestError=thisError;
				bestModel=thisModel;                 //guardem millor model
				m_maybeInliers = maybeInliers;       //guardam millors matches
				m_maybeInliersPrev = maybeInliersPrev;
			}

		}
	}
	if(m_goodProcess){
	printf("Model trobat amb %i matches bons \n", m_maybeInliers.size());
	}
	return bestModel;

}


double Motion::computeError(vector <Point2f> &features1, vector <Point2f> &features2, Data theMotion){
	unsigned int i;
		double x1, y1, x2, y2, theDistance, theError=0.0;
		for (i=0;i<features2.size();i++) {
			Data thePoint(features2[i].x, features2[i].y, 0);
			thePoint=theMotion.compose(thePoint);
			x1=thePoint.getX();
			y1=thePoint.getY();
			x2=features1[i].x;
			y2=features1[i].y;
			x1=(x1-x2)*(x1-x2);
			y1=(y1-y2)*(y1-y2);
			theDistance=sqrt(x1+y1);
			theError+=theDistance;
		}
		return theError;

}

bool Motion::goodRANSAC(){
	if (m_goodProcess){
		return TRUE;    //ha guardat com a mínim un bon model RANSAC o LS
	}
	else {
		printf("Error, cap model amb RANSAC trobat! \n");
		switch(m_optionRansac){
			case 1:
				saveOdoPrev();   //guardam l'anterior moviment
				return FALSE;
			break;
			case 2:
				printf("Minims Quadrats --> ");
				inimotion(m_features1, m_features2 ,1);   //executam mins squares
				return TRUE;
			break;

		}
	}
}



void Motion::saveOdoCurrent(double X, double Y, double O){
	m_Xprev = X; m_Yprev = Y; m_Oprev = O; //guardam com a moviment anterior el moviment actual
	
	m_odoX.push_back(X);
	m_odoY.push_back(Y);
	m_odoO.push_back(O);
	printf("X:%f  Y:%f  O:%f \n", X,Y,(O*180/3.1415926535));
}

void Motion::saveOdoPrev(){
	
	m_odoX.push_back(m_Xprev);  
	m_odoY.push_back(m_Yprev);
	m_odoO.push_back(m_Oprev);

	printf("FRAME ANTERIOR --> X:%f  Y:%f  O:%f \n", m_Xprev,m_Yprev,(m_Oprev*180/3.1415926535));
}

void Motion::compose(){
	m_composeX.push_back(0); m_composeY.push_back(0); m_composeO.push_back(0);

	for (unsigned int i =0; i<m_odoX.size(); i++){
		m_composeX.push_back(m_composeX[i]+m_odoX[i]*cos(m_composeO[i])-m_odoY[i]*sin(m_composeO[i]));
		m_composeY.push_back(m_composeY[i]+m_odoX[i]*sin(m_composeO[i])+m_odoY[i]*cos(m_composeO[i]));
		m_composeO.push_back(m_composeO[i]+m_odoO[i]);
	}
}

void Motion::writeCompose(double Time){

	prepareUrlData();

	FILE *write;
	write = fopen(m_fName,"w");
	fprintf (write, "%f %i %i \n", Time, 0, 0);
		for (unsigned int i=0;i<m_odoX.size();i++) {
			fprintf(write, "%f %f %f\n", m_composeX[i], m_composeY[i], m_composeO[i]);   //guardam l'odometria guardada en els vectors
		}	
	fclose(write);

}


void Motion::prepareUrlData(){

	char url[100];
	int i;
	for(i = 0; i <m_url.size(); i++){  //string a array de char
		url[i] = m_url[i];
	}
	url[i] = 0;

	if(m_selec3 == 1){
		sprintf(m_fName,"%s/ExpOdo/A%iB%iC%i.txt",url,m_selec3,m_xFrame,m_numMin);   //cadena de text de nom de fixer per l'odometria (Opció= LS)
	}

	if(m_selec3 == 2){
		sprintf(m_fName,"%s/ExpOdo/A%iB%iC%iD%iE%i.txt",url,m_selec3,m_xFrame,m_numMin,m_optionRansac,m_RANSAC_minConsensus);   //cadena de text de nom de fixer per l'odometria (Opció = RANSAC)
	}

}

void Motion::imprimirRANSAC(Mat img1, Mat img2){   //ATENCIÓ, aquesta funció només funciona sense passar a metros (no oblidar canviar llindar Param2)
	
	int j = 0;
	Mat img_1 = img1; Mat img_2 = img2;
	Mat img_matches_RANSAC;
	vector <KeyPoint> feat_ransac1, feat_ransac2;
	vector <DMatch> match;
	Point ptCurr, ptPrev;
	drawMatches(img_1, feat_ransac1, img_2, feat_ransac2, match, img_matches_RANSAC,  Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
		
	for (j=0;j<m_maybeInliersPrev.size();j++) {

		m_maybeInliers[j].x = m_maybeInliers[j].x + img_2.cols;
		ptCurr.x = m_maybeInliers[j].x+img_2.cols/2; ptCurr.y = m_maybeInliers[j].y+img_2.rows/2;
		ptPrev.x = m_maybeInliersPrev[j].x+img_2.cols/2; ptPrev.y = m_maybeInliersPrev[j].y+img_2.rows/2;
		line(img_matches_RANSAC, ptCurr, ptPrev,Scalar(30, 50, 350), 1.999 );
		circle(img_matches_RANSAC,ptCurr,3.9,Scalar(30, 50, 350),1.999);
		circle(img_matches_RANSAC,ptPrev,3.9,Scalar(30, 50, 350),1.999);
	}			
	imshow("Matches RANSAC", img_matches_RANSAC);
}


vector <Point2f> &Motion::getMaybeInliers(){
	return m_maybeInliers;
}

vector <Point2f> &Motion::getMaybeInliersPrev(){
	return m_maybeInliersPrev;
}