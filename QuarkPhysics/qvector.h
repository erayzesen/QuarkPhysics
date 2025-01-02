
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

#ifndef QVECTOR_H
#define QVECTOR_H
#include <cmath>
#include <ostream>
#include "qmath_utils.h"

using namespace std;

enum QSides{
	UP,
	RIGHT,
	DOWN,
	LEFT,
	NONE
};

struct QVector{
public:
	QVector(){
		QVector(0,0);
	};
	QVector(float px, float py){
		x=px;
		y=py;
	};
	float x=0;

	float y=0;

	static QVector Up(){
		return QVector(0.0f,-1.0f);
	};
	static QVector Right(){
		return QVector(1.0f,0.0f);
	};
	static QVector Down(){
		return QVector(0.0f,1.0f);
	};
	static QVector Left(){
		return QVector(-1.0f,0.0f);
	};
	static QVector Zero(){
		return QVector(-0.0f,0.0f);
	};

	static QVector NaN(){
		return QVector(NAN,NAN);
	};




	//Math Operations
	QVector &operator +=(const QVector &other) {
		this->x+=other.x;
		this->y+=other.y;
		return *this;
	};
	friend QVector operator +(const QVector pointA, const QVector pointB){
		return QVector(pointA.x+pointB.x,pointA.y+pointB.y);
	}

	QVector &operator /=(const QVector &other) {
		this->x/=other.x;
		this->y/=other.y;
		return *this;
	};
	QVector &operator /=(const float &value) {
		this->x/=value;
		this->y/=value;
		return *this;
	};
	friend QVector operator /(const QVector pointA,const QVector pointB){
		return QVector(pointA.x/pointB.x,pointA.y/pointB.y);
	}
	friend QVector operator /(const float value,const QVector point){
		return QVector(point.x/value,point.y/value);
	}
	friend QVector operator /(const QVector point,const float value){
		return QVector(point.x/value,point.y/value);
	}
	QVector &operator *=(const QVector &other){
		this->x*=other.x;
		this->y*=other.y;
		return *this;
	};

	QVector &operator *=(const float &value) {
		this->x*=value;
		this->y*=value;
		return *this;
	};
	friend QVector operator *(const QVector point, const float value){
		return QVector(point.x*value,point.y*value);
	}
	friend QVector operator *(const float value,const QVector point){
		return QVector(point.x*value,point.y*value);
	}
	QVector &operator -=(const QVector &other) {
		this->x-=other.x;
		this->y-=other.y;
		return *this;
	};
	QVector operator -() const {
		return QVector(-x,-y);
	};
	friend QVector operator -(const QVector pointA, const QVector pointB){
		return QVector(pointA.x-pointB.x,pointA.y-pointB.y);
	}

	bool operator ==(const QVector &other) const {
		return x==other.x && y==other.y;
	};
	bool operator !=(const QVector &other) const {
		return !(x==other.x && y==other.y);
	};



	friend ostream& operator<<(ostream& os,QVector const & point){
		return os<<"QVector("<< point.x<<","<<point.y<<")";
	}

	//Methods

	QVector Rotated(float radianAngle) const;
	static QVector AngleToUnitVector(const float radianAngle);
	static float AngleBetweenTwoVectors(QVector vector,QVector referenceVector);
	static QSides GetVectorSide(QVector vector,QVector referenceUpVector,float maxAngleDefiningSide=0.785398f);
	static QVector GeteBisectorUnitVector(QVector pointA,QVector pointB,QVector pointC,bool checkPointsAreCCW=false);

	//Operations

	inline float Dot(QVector with) const{
		return (x*with.x)+(y*with.y);
	}
	inline float Length() const {
		return sqrt(x*x+y*y);
	}
	inline QVector Normalized() const {
		if(x==0 && y==0)
			return QVector::Zero();
		float lsqrt=LengthSquared();
		if(lsqrt==0)
			return QVector::Zero();
		float l=sqrt(lsqrt);
		return QVector(x/l,y/l);
	}
	inline QVector Perpendicular() const {
		return QVector(y,-x);
	}
	inline float LengthSquared()const {
		return (x*x)+(y*y);
	}

	inline bool isNaN() const{
		if(isnan(x) && isnan(y) ){
			return true;
		}
		return false;
	}




};

#endif // QVECTOR_H
