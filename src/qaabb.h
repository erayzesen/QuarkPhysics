#ifndef QAABB_H
#define QAABB_H
#include "qvector.h"


class QAABB
{
	QVector minPos;
	QVector maxPos;
	QVector size;
	static float multiplier;
public:


	QAABB(){
		QAABB(minPos,maxPos);
	}

	QAABB(QVector minPosition, QVector maxPosition ){
		minPos=minPosition;
		maxPos=maxPosition;
		size=maxPos-minPos;
	};

	//General Get Methods
	QVector GetMin(){
		return minPos;
	}
	QVector GetMax(){
		return maxPos;
	}
	QVector GetSize(){
		return size;
	}


	//General Set Methods



	QAABB * SetMinMax(QVector minPosition,QVector maxPosition){
		minPos=minPosition;
		maxPos=maxPosition;
		size=maxPos-minPos;
		return this;
	}


	float GetPerimeter() const{
		return 2.0f*(size.x+size.y);
	}

	float GetArea()const{
		return size.x*size.y;
	}

	QVector GetCenterPosition()const {
		return (minPos+maxPos)*0.5f;
	}

	bool isContain(QAABB &otherAABB)const{
		bool result=true;
		result=result && minPos.x<=otherAABB.minPos.x;
		result=result && minPos.y<=otherAABB.minPos.y;
		result=result && maxPos.x>=otherAABB.maxPos.x;
		result=result && maxPos.y>=otherAABB.maxPos.y;

		return result;

	}
	static QAABB Combine( QAABB &b1, QAABB &b2){
		QAABB output=QAABB();
		float minX=b1.minPos.x<b2.minPos.x ? b1.minPos.x : b2.minPos.x;
		float minY=b1.minPos.y<b2.minPos.y ? b1.minPos.y : b2.minPos.y;

		float maxX=b1.maxPos.x>b2.maxPos.x ? b1.maxPos.x : b2.maxPos.x;
		float maxY=b1.maxPos.y>b2.maxPos.y ? b1.maxPos.y : b2.maxPos.y;
		output.SetMinMax(QVector(minX,minY),QVector(maxX,maxY) );
		return output;
	};

	void Fatten(float amount){
		QVector amountVec=QVector(amount,amount);
		SetMinMax(minPos-amountVec,maxPos+amountVec);
	}

	QAABB Fatted(float amount)const {
		QVector amountVec=QVector(amount,amount);
		return QAABB(minPos-amountVec,maxPos+amountVec);

	}
	bool isCollidingWith(QAABB &otherAABB) const{
		if( minPos.x<=otherAABB.maxPos.x && maxPos.x>=otherAABB.minPos.x && minPos.y<=otherAABB.maxPos.y && maxPos.y>=otherAABB.minPos.y ){
			return true;
		}
		return false;
	}



};

#endif // QAABB_H
