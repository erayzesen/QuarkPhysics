
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

#include "qjoint.h"
#include "qworld.h"
#include <iostream>

QJoint::QJoint(QRigidBody *bodyA,QVector anchorWorldPositionA,QVector anchorWorldPositionB,QRigidBody* bodyB)
{
	length=(anchorWorldPositionA-anchorWorldPositionB).Length();
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

QJoint *QJoint::SetCollisionEnabled(bool value) {
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

void QJoint::Update() {
	if(enabled==false){
		return;
	}

	
	if(bodyA==nullptr && bodyB==nullptr)
		return;

	if(bodyA!=nullptr){

		if(bodyA->GetEnabled()==false )
			return;
		
		if( bodyA->GetMode()==QBody::STATIC && bodyB==nullptr ){
			return;
		}
	}

	if(bodyB!=nullptr){

		if(bodyB->GetEnabled()==false )
			return;

		if( bodyB->GetMode()==QBody::STATIC  && bodyA==nullptr){
			return;
		}
	}

	float forceRatioA=balance;
	float forceRatioB=1.0f-balance;

	if(bodyA!=nullptr && bodyB!=nullptr){
		
		if(bodyA->GetMode()==QBody::STATIC && bodyB->GetMode()==QBody::STATIC){
			return;
		}
		if(bodyA->GetMode()==QBody::STATIC){
			forceRatioA=0.0f;
			forceRatioB=1.0f;
		}
		if(bodyB->GetMode()==QBody::STATIC){
			forceRatioA=1.0f;
			forceRatioB=0.0f;
		}
	}else{
		if(bodyA==nullptr){
			forceRatioA=0.0f;
			forceRatioB=1.0f;
		}
		if(bodyB==nullptr){
			forceRatioA=1.0f;
			forceRatioB=0.0f;
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
	float distanceDelta=(length-currentDistance);
	if (distanceDelta==0.0)
		return;
	if(grooveEnabled==true && currentDistance<length)
		return;
	QVector distanceDeltaVec=distanceDelta*unit;


	jointForce=distanceDeltaVec*rigidity;
	jointForceA=-jointForce*forceRatioA;
	jointForceB=jointForce*forceRatioB;


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
