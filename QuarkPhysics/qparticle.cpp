
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

#include "qparticle.h"
#include "qmesh.h"
#include "qbody.h"
#include "qvector.h"

void QParticle::ClearOneTimeCollisions()
{
	oneTimeCollidedBodies.clear();
	previousCollidedBodies.clear();
}

void QParticle::ResetOneTimeCollisions()
{
	oneTimeCollidedBodies=previousCollidedBodies;
	previousCollidedBodies.clear();
}

void QParticle::UpdateAABB()
{
	float minX=GetGlobalPosition().x-GetRadius();
	float maxX=GetGlobalPosition().x+GetRadius();

	float minY=GetGlobalPosition().y-GetRadius();
	float maxY=GetGlobalPosition().y+GetRadius();

	aabb.SetMinMax(QVector(minX,minY),QVector(maxX,maxY) );


}

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
	aabbNeedsUpdate=true;
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
			if (ownerBody->GetBodyType()==QBody::BodyTypes::SOFT){
				ownerBody->WakeUp();
			}
			ownerMesh->subConvexPolygonsNeedsUpdate=true;
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

QParticle *QParticle::SetEnabled(bool value)
{
	enabled=value;
    return this;
}

QParticle *QParticle::SetIsLazy(bool value)
{
	lazy=value;
    return this;
}

QParticle *QParticle::ApplyForce(QVector value)
{
	AddGlobalPosition(value);
	if(ownerMesh==nullptr){
		position=globalPosition;
	}
	return this;
}
QParticle *QParticle::SetForce(QVector value){
	if (ownerMesh!=nullptr){
		if (ownerMesh->GetOwnerBody()!=nullptr ){
			ownerMesh->GetOwnerBody()->WakeUp();
		}
	}
	force=value;
	return this;
}
QParticle *QParticle::AddForce(QVector value){
	return SetForce(GetForce()+value);
}

QParticle *QParticle::AddAccumulatedForce(QVector value)
{
	accumulatedForces.push_back(value);
    return this;
}

QParticle *QParticle::ClearAccumulatedForces()
{
	accumulatedForces.clear();
    return this;
}

QParticle *QParticle::ApplyAccumulatedForces()
{
	if(accumulatedForces.size()>0 ){
		QVector accumulatedForce=QVector::Zero();
		for(size_t j=0;j<accumulatedForces.size();++j ){
			accumulatedForce+=accumulatedForces[j];
		}
		accumulatedForce/=accumulatedForces.size();
		ApplyForce(accumulatedForce);
		accumulatedForces.clear();
	}
    return this;
}

bool QParticle::IsConnectedWithSpring(QParticle *particle)
{
    return springConnectedParticles.find(particle) != springConnectedParticles.end();
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

