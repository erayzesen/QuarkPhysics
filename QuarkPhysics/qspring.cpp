
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

void QSpring::Update(float rigidity,bool internalsException, bool isWorldSpring)
{
	if(enabled==false)
		return;
	if (rigidity==0.0f)
		return;

	if( pA==nullptr || pB==nullptr){
		return;
	}
	bool particleACanGetResponse=true;
	bool particleBCanGetResponse=true;

	//Check body modes and simulation models 
	if(isWorldSpring==true){
		if(pA->GetOwnerMesh()!=nullptr ){
			QBody *bA=pA->GetOwnerMesh()->GetOwnerBody();
			if(bA!=nullptr){
				if( bA->GetSimulationModel()==QBody::SimulationModels::RIGID_BODY ||  bA->GetMode()==QBody::Modes::STATIC ){
					particleACanGetResponse=false;
				}
			}
		}
		

		if(pB->GetOwnerMesh()!=nullptr ){
			QBody *bB=pB->GetOwnerMesh()->GetOwnerBody();
			if(bB!=nullptr){
				if( bB->GetSimulationModel()==QBody::SimulationModels::RIGID_BODY ||  bB->GetMode()==QBody::Modes::STATIC ){
					particleBCanGetResponse=false;
				}
			}
		}

		if (pB->GetEnabled()==false ){
			particleBCanGetResponse=false;
		}

	}

	if (pA->GetEnabled()==false ){
		particleACanGetResponse=false;
	}

	if (pB->GetEnabled()==false ){
		particleBCanGetResponse=false;
	}

	if (particleACanGetResponse==false && particleBCanGetResponse==false ){
		return;
	}



	
	
	if(pA->GetOwnerMesh()!=nullptr && pB->GetOwnerMesh()!=nullptr){
		if(pA->GetOwnerMesh()->GetOwnerBody()!=nullptr && pB->GetOwnerMesh()->GetOwnerBody()!=nullptr){
			if (pA->GetOwnerMesh()->GetOwnerBody()->GetIsSleeping() && pB->GetOwnerMesh()->GetOwnerBody()->GetIsSleeping() ){
				return;
			}
		}
	}
	QVector sv=pB->GetGlobalPosition()-pA->GetGlobalPosition(); //spring vec
	float sl=sv.Length(); //spring distance
	QVector svu=sv.Normalized(); //spring vector unit

	QVector force=(length-sl)*svu;

	QVector forceA=-force;
	QVector forceB=force;

	if(internalsException && isInternal ){
		force=(pB->GetGlobalPosition()-pA->GetGlobalPosition() )*0.5f;
		forceA=force;
		forceB=-force;

		if(pA->GetIsInternal()==true && pB->GetIsInternal()==false){
			forceA*=1.0f;
			forceB*=0;
		}else if(pA->GetIsInternal()==false && pB->GetIsInternal()==true){
			forceA*=0.0f;
			forceB*=1.0f;
		}else if(pA->GetIsInternal()==true && pB->GetIsInternal()==true){
			forceA*=0.5f;
			forceB*=0.5f;
		}else if(pA->GetIsInternal()==false && pA->GetIsInternal()==false ){
			return;
		}

		pA->AddAccumulatedForce(forceA);
		pB->AddAccumulatedForce(forceB);
		return;
	}else{
		float k=0.5f;
		if(particleACanGetResponse==false || particleBCanGetResponse==false)
			k=1.0f;
		
		if (enableDistanceLimit){
			float lengthRate=sl/length;
			if( lengthRate>maximumDistanceFactor || lengthRate<minimumDistanceFactor ){
				rigidity=1.0f;
			} 
		}
		forceA*=k*rigidity;
		forceB*=k*rigidity;
	}

	if(particleACanGetResponse){
		pA->ApplyForce(forceA);
	}
	if(particleBCanGetResponse){
		pB->ApplyForce(forceB);
	}
		

}






