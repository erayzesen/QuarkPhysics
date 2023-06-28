#include "qparticle.h"
#include "qmesh.h"
#include "qbody.h"


QParticle::QParticle()
{

}

QParticle::QParticle(float posX, float posY, float radius)
{
	position=QVector(posX,posY);
	globalPosition=QVector(posX,posY);
	prevGlobalPosition=QVector(posX,posY);
	r=radius;
}

QParticle::QParticle(QVector pos, float radius)
{
	position=QVector(pos.x,pos.y);
	globalPosition=QVector(pos.x,pos.y);
	prevGlobalPosition=QVector(pos.x,pos.y);
	r=radius;
}

QParticle *QParticle::SetGlobalPosition(QVector position){
	this->globalPosition=position;
	if(ownerMesh==nullptr){
		this->position=position;
	}
	return this;
}

QParticle *QParticle::SetPreviousGlobalPosition(QVector position){
	this->prevGlobalPosition=position;
	return this;
}

QParticle *QParticle::SetPosition(QVector position){
	this->position=position;
	if(ownerMesh==nullptr){
		this->globalPosition=position;
	}else{
		QBody* ownerBody=ownerMesh->GetOwnerBody();
		if(ownerBody!=nullptr){
			ownerBody->interiaNeedsUpdate=true;
			ownerBody->circumferenceNeedsUpdate=true;
		}
	}
	return this;
}

QParticle *QParticle::SetMass(float value){
	mass=value;
	return this;
}

QParticle *QParticle::SetOwnerMesh(QMesh *mesh){
	this->ownerMesh=mesh;
	return this;
}

QParticle *QParticle::SetRadius(float radius){
	r=radius;
	if(ownerMesh!=nullptr){
		QBody* ownerBody=ownerMesh->GetOwnerBody();
		if(ownerBody!=nullptr){
			ownerBody->interiaNeedsUpdate=true;
		}
	}
	return this;
}

QParticle *QParticle::SetIsInternal(bool value)
{
	isInternal=value;
	return this;
}

QParticle *QParticle::SetForce(QVector value)
{
	force=value;
	return this;
}

QParticle *QParticle::ApplyForce(QVector value)
{
	globalPosition+=value;
	if(ownerMesh==nullptr){
		position=globalPosition;
	}
	return this;
}

void QParticle::ApplyForceToParticleSegment(QParticle *pA, QParticle *pB,QVector force, QVector fromPosition)
{
	QVector segmentVector=pB->globalPosition-pA->globalPosition;
	QVector unit=segmentVector.Normalized();
	float len=segmentVector.Length();
	QVector bridgeVector=fromPosition-pA->globalPosition;
	float proj=bridgeVector.Dot(unit);

	float rateA;
	float rateB;
	if(proj<0 && proj>len){
		rateA=0.5f;
		rateB=0.5f;
	}else{
		float u=1/len;
		rateA=(len-proj)*u;
		rateB=proj*u;
	}

	pA->globalPosition+=rateA*force;
	pB->globalPosition+=rateB*force;
}

QParticle* QParticle::AddForce(QVector value)
{
	this->force=value;
	return this;
}
