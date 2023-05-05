
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

#include "qspring.h"
#include <iostream>
#include "qmesh.h"
#include "qbody.h"

QSpring::QSpring(QParticle *particleA, QParticle *particleB, bool internal)
{
	pA=particleA;
	pB=particleB;
	length=(pA->GetGlobalPosition()-pB->GetGlobalPosition()).Length();
	isInternal=internal;

}

QSpring::QSpring(QParticle *particleA, QParticle *particleB, float length, bool internal)
{
	pA=particleA;
	pB=particleB;
	this->length=length;
	isInternal=internal;
}

void QSpring::Update(float rigidity,bool internalsException)
{
	bool isBodiesIsSleeping=false;
	if(pA->GetOwnerMesh()!=nullptr and pB->GetOwnerMesh()!=nullptr){
		if(pA->GetOwnerMesh()->GetOwnerBody()!=nullptr and pB->GetOwnerMesh()->GetOwnerBody()!=nullptr){
			if (pA->GetOwnerMesh()->GetOwnerBody()->GetIsSleeping() && pB->GetOwnerMesh()->GetOwnerBody()->GetIsSleeping() ){
				return;
			}
		}
	}
	if(enabled==false)
		return;
	QVector sv=pB->GetGlobalPosition()-pA->GetGlobalPosition(); //spring vec
	float sl=sv.Length(); //spring distance
	QVector svu=sv.Normalized(); //spring vector unit

	QVector force=(length-sl)*svu;

	QVector forceA=-force;
	QVector forceB=force;

	if(internalsException && isInternal ){

		if(pA->GetIsInternal()==true && pB->GetIsInternal()==false){
			forceA*=sl<length ? 0:0.5f;
			forceB*=0;
		}else if(pA->GetIsInternal()==false && pB->GetIsInternal()==true){
			forceA*=0.0f;
			forceB*=sl<length ? 0:0.5f;
		}else if(pA->GetIsInternal()==true && pB->GetIsInternal()==true){
			forceA*=sl<length ? 0:0.25f;
			forceB*=sl<length ? 0:0.25f;
		}
	}else{
		forceA*=0.5f*rigidity;
		forceB*=0.5f*rigidity;
	}




	pA->ApplyForce(forceA);
	pB->ApplyForce(forceB);

}






