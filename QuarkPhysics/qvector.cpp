#include "qvector.h"
#include <cmath>







QVector QVector::AngleToUnitVector(const float radianAngle){
	return QVector( cos(radianAngle),sin(radianAngle) );
}

float QVector::AngleBetweenTwoVectors(QVector vector, QVector referenceVector)
{
	float totalLength=vector.Length()+referenceVector.Length();

	QVector refPerp=referenceVector.Perpendicular();

	float dot=vector.Dot(referenceVector);
	float perpDot=vector.Dot(refPerp);

	float cosA=0.0f;
	float sinA=0.0f;

	if(totalLength!=0){
		cosA=dot/totalLength;
		sinA=perpDot/totalLength;
	}

	float aSin;

	if(sinA<-1.0f){
		aSin=asin(-1.0);
	}else if(sinA>1.0f){
		aSin=asin(1.0);
	}else{
		aSin=asin(sinA);
	}

	float rad=-atan2(aSin,cosA);

	return rad;


}

QSides QVector::GetVectorSide(QVector vector, QVector referenceUpVector, float maxAngleDefiningSide)
{
	float ang=AngleBetweenTwoVectors(vector,referenceUpVector);

	if(abs(ang)<maxAngleDefiningSide){
		return QSides::UP;
	}else if(ang>M_PI_2-maxAngleDefiningSide && ang<M_PI_2+maxAngleDefiningSide){
		return QSides::RIGHT;
	}else if(ang<-(M_PI_2-maxAngleDefiningSide) && ang>-(M_PI_2+maxAngleDefiningSide)){
		return QSides::LEFT;
	}else if(abs(ang)>M_PI-maxAngleDefiningSide){
		return QSides::DOWN;
	}

	return QSides::NONE;
}

QVector QVector::Rotated(float radianAngle) const{
	if(radianAngle==0)return QVector(x,y);
	QVector rotVec=AngleToUnitVector(radianAngle);
	float nx=x*rotVec.x-y*rotVec.y;
	float ny=x*rotVec.y+y*rotVec.x;
	return QVector(nx,ny);
}
