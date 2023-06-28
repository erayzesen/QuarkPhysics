#include "qvector.h"
#include <cmath>







QVector QVector::AngleToUnitVector(const float radianAngle){
	return QVector( cos(radianAngle),sin(radianAngle) );
}

QVector QVector::Rotated(float radianAngle) const{
	if(radianAngle==0)return QVector(x,y);
	QVector rotVec=AngleToUnitVector(radianAngle);
	float nx=x*rotVec.x-y*rotVec.y;
	float ny=x*rotVec.y+y*rotVec.x;
	return QVector(nx,ny);
}
