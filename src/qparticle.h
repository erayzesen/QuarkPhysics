#ifndef QPARTICLE_H
#define QPARTICLE_H
#include "qvector.h"

class QMesh;

class QParticle
{
	float r=0.5f;

	QVector globalPosition;
	QVector prevGlobalPosition;

	QVector position;

	float mass=1.0f;

	QMesh *ownerMesh=nullptr;

	bool isInternal=false;

	QVector force=QVector::Zero();
public:
	QParticle();
	QParticle(float posX,float posY,float radius=0.5f);
	QParticle(QVector pos,float radius=0.5f);




	//Get Methods
	QVector GetGlobalPosition(){
		return globalPosition;
	}
	QVector GetPreviousGlobalPosition(){
		return prevGlobalPosition;
	}
	QVector GetPosition(){
		return position;
	}
	float GetMass(){
		return mass;
	}
	QMesh * GetOwnerMesh(){
		return ownerMesh;
	}
	float GetRadius(){
		return r;
	}
	float GetIsInternal(){
		return isInternal;
	}

	QVector GetForce(){
		return force;
	}

	//Set Methods
	QParticle *SetGlobalPosition(QVector position);
	QParticle *SetPreviousGlobalPosition(QVector position);
	QParticle *SetPosition(QVector position);
	QParticle *SetMass(float value);
	QParticle *SetOwnerMesh(QMesh *mesh);
	QParticle *SetRadius(float radius);
	QParticle *SetIsInternal(bool value);
	QParticle *SetForce(QVector value);

	//
	QParticle *ApplyForce(QVector value);
	QParticle *AddForce(QVector value);

	//Static Methods
	static void ApplyForceToParticleSegment(QParticle *pA,QParticle *pB,QVector force,QVector fromPosition);
};

#endif // QPARTICLE_H
