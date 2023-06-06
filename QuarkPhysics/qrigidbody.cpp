
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

#include "qrigidbody.h"
#include "qvector.h"
#include "qworld.h"
#include "qmesh.h"


QRigidBody::QRigidBody()
{
	simulationModel=SimulationModels::RIGID_BODY;
	bodyType=QBody::BodyTypes::RIGID;
}


QRigidBody *QRigidBody::SetPositionAndCollide(QVector value,bool withPreviousPosition){
	SetPosition(value,withPreviousPosition);
	if(world!=nullptr){
		world->CollideWithWorld(this);
	}
	return this;

}

QRigidBody *QRigidBody::ApplyForce(QVector force, QVector r,bool updateMeshTransforms)
{
	position+=force;

	if(fixedRotation==false){
		rotation+=r.Dot(force.Perpendicular())/GetInertia();
	}
	if(updateMeshTransforms==true)
		UpdateMeshTransforms();


	return this;
}

QRigidBody *QRigidBody::ApplyImpulse(QVector impulse, QVector r)
{

	prevPosition-=impulse;

	if(fixedRotation==false){
		float angVel=r.Dot(impulse.Perpendicular())/GetInertia();
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



	if(isSleeping){
		return;
	}

	float ts=1.0f;

	if(enableBodySpecificTimeScale==true){
		ts=bodySpecificTimeScale;
	}else{
		if(world!=nullptr){
			ts=world->GetTimeScale();
		}
	}


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


	//Verlet Integration
	if(isKinematic==false){
		position+=vel;
		position+=(world->GetGravity()*mass*ts);
		rotation+=rotVel;
	}
	//Position Forces
	position+=force;
	force=QVector::Zero();
	//Angular Forces
	rotation+=angularForce;
	angularForce=0.0f;

	UpdateMeshTransforms();
	UpdateAABB();
}


