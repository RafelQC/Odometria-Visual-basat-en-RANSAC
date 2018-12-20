#include "Data.h"

Data::Data(double x, double y, double o) {
	m_x=x;
	m_y=y;
	m_o=o;
}

double Data::getX(){
	return m_x;
}

double Data::getY(){
	return m_y;
}

double Data::getO(){
	return m_o;
}

Data Data::compose(Data thePose){  // composam el moviment
	Data X;
	X.m_x=m_x+thePose.m_x*cos(m_o)-thePose.m_y*sin(m_o);
	X.m_y=m_y+thePose.m_x*sin(m_o)+thePose.m_y*cos(m_o);
	X.m_o=m_o+thePose.m_o;
	return X;
}