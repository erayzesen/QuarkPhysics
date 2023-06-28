#ifndef QVECTOR_H
#define QVECTOR_H
#include <cmath>
#include <ostream>

using namespace std;
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
