#include "qjoint.h"
#include "qworld.h"
#include <iostream>

QJoint::QJoint(QRigidBody *bodyA,QVector anchorWorldPositionA,QVector anchorWorldPositionB,QRigidBody* bodyB)
{
	distance=(anchorWorldPositionA-anchorWorldPositionB).Length();
	//Initialize A
	this->bodyA=bodyA;

	if(bodyA!=nullptr){
		this->anchorA=(anchorWorldPositionA-bodyA->GetPosition()).Rotated(-bodyA->GetRotation());
	}else{
		this->anchorA=anchorWorldPositionA;
	}
	this->anchorGlobalA=anchorWorldPositionA;

	//Initialize B
	this->bodyB=bodyB;

	if(bodyB!=nullptr){
		this->anchorB=(anchorWorldPositionB-bodyB->GetPosition()).Rotated(-bodyB->GetRotation());
	}else{
		this->anchorB=anchorWorldPositionB;
	}
	this->anchorGlobalB=anchorWorldPositionB;

	if(bodyA!=nullptr){
		if(bodyA->GetWorld()!=nullptr)
			world=bodyA->GetWorld();
	}
	if(world==nullptr && bodyB!=nullptr)
		if(bodyB->GetWorld()!=nullptr)
			world=bodyB->GetWorld();

	SetCollisionEnabled(false);
}

QJoint::~QJoint()
{
	if(collisionsEnabled==false && world!=nullptr && bodyA!=nullptr && bodyB!=nullptr)
		world->RemoveCollisionException(bodyA,bodyB);
}



void QJoint::Update()
{



	if(bodyA!=nullptr){
		if(bodyA->GetMode()==QBody::STATIC && bodyB==nullptr){
			return;
		}
	}

	if(bodyB!=nullptr){
		if(bodyB->GetMode()==QBody::STATIC && bodyA==nullptr){
			return;
		}
	}

	float k=0.5f;

	if(bodyA!=nullptr && bodyB!=nullptr){
		if(bodyA->GetMode()==QBody::STATIC && bodyB->GetMode()==QBody::STATIC){
			return;
		}
		if(bodyA->GetMode()==QBody::STATIC || bodyB->GetMode()==QBody::STATIC){
			k=1.0f;
		}
	}else{
		if(bodyA!=nullptr || bodyB!=nullptr){
			k=1.0f;
		}
	}

	QVector anchorTransformedA=anchorA;
	QVector anchorTransformedB=anchorB;






	if(bodyA!=nullptr){
		anchorTransformedA=anchorA.Rotated(bodyA->GetRotation());
		anchorGlobalA=bodyA->GetPosition()+anchorTransformedA;

	}else{
		anchorGlobalA=anchorA;
	}

	if(bodyB!=nullptr){
		anchorTransformedB=anchorB.Rotated(bodyB->GetRotation());
		anchorGlobalB=bodyB->GetPosition()+anchorTransformedB;

	}else{
		anchorGlobalB=anchorB;
	}




	QVector diff=QVector::Zero();
	QVector jointForce=QVector::Zero();
	QVector jointForceA=QVector::Zero();
	QVector jointForceB=QVector::Zero();

	diff=anchorGlobalB-anchorGlobalA;
	float currentDistance=diff.Length();
	QVector unit=diff.Normalized();
	float distanceDelta=(distance-currentDistance);
	if(grooveEnabled==true && currentDistance<distance)
		return;
	QVector distanceDeltaVec=distanceDelta*unit;


	jointForce=distanceDeltaVec*rigidity*k;
	jointForceA=-jointForce;
	jointForceB=jointForce;


	if(bodyA!=nullptr){
		if(bodyA->GetMode()!=QBody::STATIC){
			bodyA->ApplyForce(jointForceA,anchorTransformedA);
		}
	}

	if(bodyB!=nullptr){
		if(bodyB->GetMode()!=QBody::STATIC){
			bodyB->ApplyForce(jointForceB,anchorTransformedB);

		}
	}
}


QJoint *QJoint::SetCollisionEnabled(bool value){
	collisionsEnabled=value;
	if(world!=nullptr && bodyA!=nullptr && bodyB!=nullptr){
		if(value==true){
			world->RemoveCollisionException(bodyA,bodyB);
		}else{
			world->AddCollisionException(bodyA,bodyB);
		}
	}
	return this;
}
