#include "qspring.h"
#include <iostream>

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






