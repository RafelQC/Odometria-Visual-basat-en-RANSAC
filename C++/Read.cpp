#include "Read.h"

Mat &Read::getMatrix1(){
return m_img1;
}

Mat &Read::getMatrix2(){
return m_img2;
}

Mat &Read::nextImg(int counter){
	char fName[100];
	sprintf(fName,"%s/i%04d.jpg",m_url, counter);
	m_next_img = imread(fName, CV_LOAD_IMAGE_GRAYSCALE);
	return m_next_img;
}

bool Read::moreFrames(int counter){
	struct stat   buffer;
	char fName[100];
	sprintf(fName,"%s/i%04d.jpg",m_url, counter);
	return (stat (fName, &buffer) == 0);
}

void Read::iniread(string url){
	int i;
	for(i = 0; i <url.size(); i++){
		m_url[i] = url[i];
	}
	m_url[i] = 0;
}