
/************************************************************************************
 * MIT License
 *
 * Copyright (c) 2023 Eray Zesen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * https://github.com/erayzesen/QuarkPhysics
 *
**************************************************************************************/


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

QVector QVector::GeteBisectorUnitVector(QVector pointA,QVector pointB,QVector pointC,bool checkPointsAreCCW)
{
	QVector fromPrev=pointB-pointA;
	QVector toNext=pointC-pointB;


	QVector prevToNext=pointC-pointA;
	QVector prevToNextPerp=prevToNext.Perpendicular();

	QVector bisectorUnit=prevToNextPerp.Normalized();


	if(fromPrev.Dot(prevToNextPerp)<0.0f ){
		if(checkPointsAreCCW){
			QVector toCenterPos=prevToNext*0.5-fromPrev;
			if( toCenterPos.Dot(bisectorUnit)<0.0f ){
				bisectorUnit*=-1;
			}
		}
	}else{
		if (checkPointsAreCCW){
			QVector toCenterPos=prevToNext*0.5-fromPrev;
			if( toCenterPos.Dot(bisectorUnit)>0.0f ){
				bisectorUnit*=-1;
			}
		}else{
			bisectorUnit*=1;
		}
	}
	
	return -bisectorUnit;
}

QVector QVector::Rotated(float radianAngle) const{
	if(radianAngle==0)return QVector(x,y);
	QVector rotVec=AngleToUnitVector(radianAngle);
	float nx=x*rotVec.x-y*rotVec.y;
	float ny=x*rotVec.y+y*rotVec.x;
	return QVector(nx,ny);
}
