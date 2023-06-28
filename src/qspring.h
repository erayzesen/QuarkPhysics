#ifndef QSPRING_H
#define QSPRING_H
#include "qparticle.h"

class QMesh;

class QSpring
{
	float rigidity=1.0f;
	QParticle *pA;
	QParticle *pB;
	float length;
	bool isInternal=false;
	bool enabled=true;
public:
	QSpring(QParticle *particleA,QParticle *particleB,bool internal=false);
	QSpring(QParticle *particleA,QParticle *particleB,float length,bool internal=false);




	virtual void Update(float rigidity,bool internalsException);

	//Get Methods
	QParticle *GetParticleA(){
		return pA;
	}
	QParticle *GetParticleB(){
		return pB;
	}
	float GetLength(){
		return length;
	}
	bool GetIsInternal(){
		return isInternal;
	}
	float GetRigidity(){
		return rigidity;
	}
	bool GetEnabled(){
		return enabled;
	}

	//Set Methods
	QSpring *SetParticleA(QParticle *particle){
		pA=particle;
		return this;
	}
	QSpring *SetParticleB(QParticle *particle){
		pB=particle;
		return this;
	}
	QSpring *SetLength(float length){
		this->length=length;
		return this;
	}
	QSpring *SetIsInternal(bool value){
		isInternal=true;
		return this;
	}
	QSpring *SetRigidity(float rigidity){
		this->rigidity=rigidity;
		return this;
	}
	QSpring *SetEnabled(bool value){
		enabled=value;
		return this;
	}





};

#endif // QSPRING_H
