#include "qbody.h"
#include <cmath>
#include <iostream>
#include "qmesh.h"
#include "qworld.h"




QBody::QBody(){


}

QBody::~QBody()
{
	for(int i=0;i<_meshes.size();i++){
		delete _meshes[i];
	}
	_meshes.clear();
}




QVector QBody::ComputeFriction(QBody *bodyA, QBody *bodyB, QVector &normal,float penetration, QVector &relativeVelocity){

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

bool QBody::CanCollide(QBody *bodyA, QBody *bodyB)
{
	if(bodyA->world!=bodyB->world)return false;
	QWorld *world=bodyA->world;
	//Check Modes
	if(bodyA->mode==QBody::Modes::STATIC && bodyB->mode==QBody::Modes::STATIC)
		return false;
	//Check Layers Bits
	if( ( ((bodyA->layersBit & bodyB->collidableLayersBit)==0 ) && (bodyB->layersBit & bodyA->collidableLayersBit)==0 ) )
		return false;
	if(bodyA->isSleeping && bodyB->isSleeping)
		return false;

	if( world->CheckCollisionException(bodyA,bodyB)==true ){
		return false;
	}

	return true;

}


//########### GENERAL METHODS		###########









QBody * QBody::AddMesh(QMesh *mesh){
	_meshes.push_back(mesh);
	mesh->ownerBody=this;
	UpdateMeshTransforms();
	interiaNeedsUpdate=true;
	circumferenceNeedsUpdate=true;
	mesh->UpdateCollisionBehavior();
	return this;
}


QBody * QBody::RemoveMeshAt(int index){
	_meshes.erase(_meshes.begin()+index );
	interiaNeedsUpdate=true;
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
	QVector rotVecUnit=QVector::AngleToUnitVector(rotation);
	for(int n=0;n<_meshes.size();n++){
		QMesh* mesh=_meshes[n];
		mesh->globalPosition=position+mesh->position.Rotated(rotation);
		for(int i=0;i<mesh->GetParticleCount();i++){
			QParticle * particle=mesh->GetParticle(i);
			QVector originVec=particle->GetPosition();
			float nx=originVec.x*rotVecUnit.x-originVec.y*rotVecUnit.y;
			float ny=originVec.y*rotVecUnit.x+originVec.x*rotVecUnit.y;
			QVector newPos=mesh->globalPosition+QVector(nx,ny);
			particle->SetGlobalPosition(newPos);
			particle->SetPreviousGlobalPosition(newPos);
		}
	}

}



bool QBody::CanGiveCollisionResponseTo(QBody *otherBody)
{
	if(otherBody->mode==QBody::Modes::STATIC)return false;


	return true;
}


