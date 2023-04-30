#include "qparticle.h"
#include "qmesh.h"
#include "qbody.h"
#include "qvector.h"


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
QParticle *QParticle::SetGlobalPosition(QVector value){
	this->globalPosition=value;
	if(ownerMesh==nullptr){
		this->position=this->globalPosition;
	}else{
		QBody* ownerBody=ownerMesh->GetOwnerBody();
		if(ownerBody!=nullptr){
			ownerBody->inertiaNeedsUpdate=true;
			ownerBody->circumferenceNeedsUpdate=true;
		}
	}
	return this;
}
QParticle *QParticle::AddGlobalPosition(QVector value){
	return SetGlobalPosition(GetGlobalPosition()+value );
}
QParticle *QParticle::SetPreviousGlobalPosition(QVector value){
	this->prevGlobalPosition=value;
	return this;
}

QParticle *QParticle::AddPreviousGlobalPosition(QVector value){
	return SetPreviousGlobalPosition(GetPreviousGlobalPosition()+value);
}

QParticle *QParticle::SetPosition(QVector value){
	this->position=value;
	if(ownerMesh==nullptr){
		this->globalPosition=position;
	}else{
		QBody* ownerBody=ownerMesh->GetOwnerBody();
		if(ownerBody!=nullptr){
			ownerBody->inertiaNeedsUpdate=true;
			ownerBody->circumferenceNeedsUpdate=true;
		}
	}
	return this;
}
QParticle *QParticle::AddPosition(QVector value){
	return SetPosition(GetPosition()+value);
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
			ownerBody->inertiaNeedsUpdate=true;
		}
	}
	return this;
}

QParticle *QParticle::SetIsInternal(bool value)
{
	isInternal=value;
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
QParticle *QParticle::SetForce(QVector value){
	force=value;
	return this;
}
QParticle *QParticle::AddForce(QVector value){
	return SetForce(GetForce()+value);
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

