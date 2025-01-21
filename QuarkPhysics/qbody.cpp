
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
#include "qbody.h"
#include <cmath>
#include <iostream>
#include <vector>
#include "qmesh.h"
#include "qworld.h"




QBody::QBody(){


}

QBody::~QBody()
{
	for(int i=0;i<_meshes.size();i++){
		if (_meshes[i]!=nullptr){
			if (_meshes[i]->manualDeletion==false){
				delete _meshes[i];
				_meshes[i]=nullptr;
			}
		}
	}
	_meshes.clear();
}

float QBody::GetVelocityLimit() {
	return velocityLimit;
}

bool QBody::GetIntegratedVelocitiesEnabled() {
	return enableIntegratedVelocities;
}

bool QBody::GetCustomGravityEnabled()
{
    return enableCustomGravity;
}

QVector QBody::GetCustomGravity()
{
    return customGravity;
}



QBody *QBody::SetVelocityLimit(float value)
{
	velocityLimit=value;
    return this;
}

QVector QBody::ComputeFriction(QBody *bodyA, QBody *bodyB, QVector &normal, float penetration, QVector &relativeVelocity)
{

    QVector tangent=relativeVelocity-(relativeVelocity.Dot(normal)*normal );

	tangent=tangent.Normalized();


	float jt=relativeVelocity.Dot(-tangent);


	float dynamicFriction=bodyA->friction<bodyB->friction ? bodyA->friction:bodyB->friction;
	float sFriction=sqrt(bodyA->staticFriction*bodyB->staticFriction);


	QVector frictionForce=QVector::Zero();

	if(std::abs(jt)<penetration*sFriction ){
		frictionForce=tangent*jt;
	}else{
		frictionForce=tangent*(-penetration)*dynamicFriction;
	}


	return frictionForce;
}

bool QBody::CanCollide(QBody *bodyA, QBody *bodyB,bool checkBodiesAreEnabled)
{
	if(bodyA->world!=bodyB->world)return false;

	if (checkBodiesAreEnabled==true)
		if (bodyA->GetEnabled()==false || bodyB->GetEnabled()==false)
			return false;

	
	//Check Static and Sleeping Modes
	if((bodyA->isSleeping || bodyA->mode==QBody::Modes::STATIC) && (bodyB->isSleeping || bodyB->mode==QBody::Modes::STATIC ))
		return false;
	//Check Layers Bits
	if( ( ((bodyA->layersBit & bodyB->collidableLayersBit)==0 ) && (bodyB->layersBit & bodyA->collidableLayersBit)==0 ) )
		return false;

	QWorld *world=bodyA->world;

	if( world->CheckCollisionException(bodyA,bodyB)==true ){
		return false;
	}

	return true;

}


//GENERAL METHODS

QBody *QBody::SetIntegratedVelocitiesEnabled(bool value) {
	enableIntegratedVelocities=value;
	return this;
}

QBody *QBody::SetCustomGravityEnabled(bool value)
{
	enableCustomGravity=value;
    return this;
}

QBody *QBody::SetCustomGravity(QVector value)
{
	customGravity=value;
    return this;
}

QBody *QBody::AddMesh(QMesh *mesh) {
	_meshes.push_back(mesh);
	mesh->ownerBody=this;
	UpdateMeshTransforms();
	inertiaNeedsUpdate=true;
	circumferenceNeedsUpdate=true;
	mesh->UpdateCollisionBehavior();
	return this;
}

QBody * QBody::AddMeshesFromFile(string filePath){
	vector<QMesh::MeshData> meshDataList=QMesh::GetMeshDatasFromFile(filePath);
	for(int i=0;i<meshDataList.size();i++){
		AddMesh(QMesh::CreateWithMeshData(meshDataList[i] ));
	}
	return this;
}


QBody * QBody::RemoveMeshAt(int index){
	_meshes.erase(_meshes.begin()+index );
	inertiaNeedsUpdate=true;
	circumferenceNeedsUpdate=true;
	return this;
}





QMesh * QBody::GetMeshAt(int index){
	return _meshes[index];
}
vector<QMesh*> * QBody::GetMeshes(){
	return &_meshes;
}

int QBody::GetMeshCount(){
	return _meshes.size();
}

void QBody::UpdateAABB(){

	float minX=QWorld::MAX_WORLD_SIZE;
	float minY=QWorld::MAX_WORLD_SIZE;
	float maxX=-QWorld::MAX_WORLD_SIZE;
	float maxY=-QWorld::MAX_WORLD_SIZE;

	for(int i=0;i<_meshes.size();i++){
	   QMesh* shape=_meshes[i];
	   for(int n=0;n<shape->particles.size();n++){
		   QParticle *particle=shape->particles[n];
		   float  r=particle->GetRadius()>0.5f ? particle->GetRadius():0;
		   float pMinX=particle->GetGlobalPosition().x-r;
		   float pMinY=particle->GetGlobalPosition().y-r;
		   float pMaxX=particle->GetGlobalPosition().x+r;
		   float pMaxY=particle->GetGlobalPosition().y+r;

		   if(pMinX<minX)
			   minX=pMinX;
		   if(pMinY<minY)
			   minY=pMinY;
		   if(pMaxX>maxX)
			   maxX=pMaxX;
		   if(pMaxY>maxY)
			   maxY=pMaxY;
	   }

   }
   aabb.SetMinMax(QVector(minX,minY),QVector(maxX,maxY) );

}

void QBody::UpdateMeshTransforms(){
	//Transforming mesh particle positions according to self rotation
	
	for(int n=0;n<_meshes.size();n++){
		QMesh* mesh=_meshes[n];
		mesh->globalRotation=rotation+mesh->rotation;
		QVector rotVecUnit=QVector::AngleToUnitVector(mesh->globalRotation);
		mesh->globalPosition=position+mesh->position.Rotated(rotation);
		for(int i=0;i<mesh->GetParticleCount();i++){
			QParticle * particle=mesh->GetParticleAt(i);
			QVector originVec=particle->GetPosition();
			float nx=originVec.x*rotVecUnit.x-originVec.y*rotVecUnit.y;
			float ny=originVec.y*rotVecUnit.x+originVec.x*rotVecUnit.y;
			QVector newPos=mesh->globalPosition+QVector(nx,ny);
			if (bodyType==QBody::BodyTypes::RIGID){
				particle->SetPreviousGlobalPosition(particle->GetGlobalPosition());
			}else{
				particle->SetPreviousGlobalPosition(newPos);
			}
			particle->SetGlobalPosition(newPos);
			
		}
	}

}

void QBody::Update()
{
	for (auto mesh: _meshes){
		for(size_t i=0;i<mesh->GetParticleCount();++i ){
			QParticle *particle=mesh->GetParticleAt(i);
			if (particle->GetIsLazy()==true ){
				particle->ResetOneTimeCollisions();
			}
		}
	}
}

bool QBody::CanGiveCollisionResponseTo(QBody *otherBody)
{
	if(otherBody->mode==QBody::Modes::STATIC)return false;
	if( otherBody->isKinematic==true && isKinematic==true && otherBody->allowKinematicCollisions==false) return false;
	if(mode!=QBody::Modes::STATIC && otherBody->isKinematic==true && isKinematic==false) return false;

	return true;
}


