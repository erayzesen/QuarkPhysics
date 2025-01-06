
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

#ifndef QAABB_H
#define QAABB_H
#include "qvector.h"
#include <vector>

using namespace std;

class QParticle;

class QAABB
{
	QVector minPos;
	QVector maxPos;
	QVector size;
	float volume;
	static float multiplier;
public:


	QAABB(){
		QAABB(minPos,maxPos);
	}

	QAABB(QVector minPosition, QVector maxPosition ){
		minPos=minPosition;
		maxPos=maxPosition;
		size=maxPos-minPos;
		volume=size.x*size.y;
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
		volume=size.x*size.y;
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

	float GetVolume() const{
		return volume;
	}

	double isContain(const QAABB& otherAABB) const {
		return (minPos.x <= otherAABB.minPos.x) &&
			(minPos.y <= otherAABB.minPos.y) &&
			(maxPos.x >= otherAABB.maxPos.x) &&
			(maxPos.y >= otherAABB.maxPos.y);
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

	QAABB Fattened(float amount)const{
		QVector amountVec=QVector(amount,amount);
		return QAABB(minPos-amountVec,maxPos+amountVec);

	}

	QAABB FattenedWithRate(float rate)const{
		QVector ratedSize=size*rate*0.5f;
		return QAABB(minPos-ratedSize,maxPos+ratedSize);
	}

	bool isCollidingWith(const QAABB& otherAABB) const {
		return maxPos.x >= otherAABB.minPos.x && minPos.x <= otherAABB.maxPos.x &&
			maxPos.y >= otherAABB.minPos.y && minPos.y <= otherAABB.maxPos.y;
	}

	static QAABB GetAABBFromParticles(vector<QParticle*> &particleCollection);



};

#endif // QAABB_H
