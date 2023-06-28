#include "qrigidbody.h"
#include "qworld.h"
#include "qmesh.h"
QRigidBody::QRigidBody()
{
	simulationModel=SimulationModels::RIGID_BODY;
}


QRigidBody *QRigidBody::ApplyForce(QVector force, QVector r,bool updateMeshTransforms)
{
	position+=force;

	if(fixedRotation==false){
		rotation+=r.Dot(force.Perpendicular())/GetInteria();
	}
	if(updateMeshTransforms==true)
		UpdateMeshTransforms();


	return this;
}

QRigidBody *QRigidBody::ApplyImpulse(QVector impulse, QVector r)
{

	prevPosition-=impulse;

	if(fixedRotation==false){
		float angVel=r.Dot(impulse.Perpendicular())/GetInteria();
		prevRotation-=angVel;

	}

	return this;
}

void QRigidBody::Update(){
	if(mode==QBody::STATIC){
		return;
	}
	if(world==nullptr){
		return;
	}

	if (isSleeping)
		return;


	QVector vel=position-prevPosition;
	prevPosition=position;
	//Reducing Float Errors
	if(abs(vel.x)<0.01){
		vel.x=0;
		prevPosition.x=position.x;
	}
	if(abs(vel.y)<0.01){
		vel.y=0;
		prevPosition.y=position.y;
	}

	//Angular Velocity
	float rotVel=rotation-prevRotation;
	prevRotation=rotation;

	//Sleep Feature
	if(canSleep){
		if(fixedVelocityTick>sleepTick && fixedAngularTick>sleepTick){
			isSleeping=true;
			prevPosition=position;
			vel=QVector::Zero();
		}else{
			isSleeping=false;
		}
	}else{
		isSleeping=false;
	}

	if(isSleeping){
		return;
	}

	//Verlet Integration
	position=position+vel;
	position+=(world->GetGravity()*mass);

	rotation+=rotVel;
	UpdateMeshTransforms();
	UpdateAABB();
}


