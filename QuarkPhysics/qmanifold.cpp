
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

#include "qmanifold.h"
#include "qareabody.h"
#include "qbody.h"
#include "qrigidbody.h"
#include "qworld.h"
#include <iostream>
#include "qmesh.h"
#include "cmath"



QManifold::QManifold(QBody *bodyA, QBody *bodyB)
{
	if(bodyA<bodyB){
		this->bodyA=bodyA;
		this->bodyB=bodyB;
	}else{
		this->bodyA=bodyB;
		this->bodyB=bodyA;
	}

	restitution=min(bodyA->restitution,bodyB->restitution);
	//restitution=1.0f;

	//One time Computed Properties

	invMass=1/(bodyA->GetMass()+bodyB->GetMass());

	if(bodyA->CanGiveCollisionResponseTo(bodyB)==false || bodyB->CanGiveCollisionResponseTo(bodyA)==false ){
		isCollisionOneSide=true;
	}



}

//void QManifold::Update(vector<QCollision::Contact> newContacts)
//{

//	//contacts.clear();
//	vector<QCollision::Contact> mergedContacts;
//	bool isFlipFlop=false;
//	for(int i=0;i<newContacts.size();i++){
//		auto nc=newContacts[i];
//		int k=-1;
//		for(int n=0;n<contacts.size();n++){
//			auto c=contacts[n];
//			if(c.particle==nc.particle){

//			}

//		}
//		mergedContacts.push_back(nc);



//	}
//	contacts=mergedContacts;
//}



QVector QManifold::GetRelativeVelocity(QParticle *contactParticle,vector<QParticle*> referenceParticles,QVector rRef, QVector rInc)
{

	QBody *bodyRef=referenceParticles[0]->GetOwnerMesh()->GetOwnerBody();
	QBody *bodyInc=contactParticle->GetOwnerMesh()->GetOwnerBody();

	QVector velRef=QVector::Zero();
	QVector velInc=QVector::Zero();
	float angVelRef=0;
	float angVelInc=0;

	if(bodyRef->GetSimulationModel()==QBody::RIGID_BODY ){
		//For RigidBodies
		angVelRef=bodyRef->GetRotation()-bodyRef->GetPreviousRotation();
		velRef=bodyRef->GetPosition()-bodyRef->GetPreviousPosition();
	}else{
		//For SoftBodies
		for(int i=0;i<referenceParticles.size();i++){
			auto p=referenceParticles[i];
			velRef+=p->GetGlobalPosition()-p->GetPreviousGlobalPosition();
		}
		velRef/=referenceParticles.size();

	}

	if( bodyInc->GetSimulationModel()==QBody::SimulationModels::RIGID_BODY  ){
		//For RigidBodies
		angVelInc=bodyInc->GetRotation()-bodyInc->GetPreviousRotation();
		velInc=bodyInc->GetPosition()-bodyInc->GetPreviousPosition();
	}else{
		//For SoftBodies
		velInc=contactParticle->GetGlobalPosition()-contactParticle->GetPreviousGlobalPosition();
	}


	return (velRef+angVelRef*-rRef.Perpendicular() )-( velInc + angVelInc*-rInc.Perpendicular() );
}





void QManifold::Solve()
{


	bool betweenRigidbodies=false;
	bool betweenSoftbodies=false;
	if(bodyA->GetSimulationModel()==QBody::SimulationModels::RIGID_BODY && bodyB->GetSimulationModel()==QBody::SimulationModels::RIGID_BODY)
		betweenRigidbodies=true;
	if(bodyA->GetSimulationModel()==QBody::SimulationModels::MASS_SPRING && bodyB->GetSimulationModel()==QBody::SimulationModels::MASS_SPRING)
		betweenSoftbodies=true;

	


	for(int i=0;i<contacts.size();i++){
		QCollision::Contact *contact=&contacts[i];

		if(betweenRigidbodies){
			contact->penetration*=0.75f;
		}else if(betweenSoftbodies){
			contact->penetration*=0.5f;
		}

		contact->penetration=max(contact->penetration,0.0f);

		QVector responseForce=contact->normal*(contact->penetration);
		if(restitution>0.0f)
			responseForce*=2.0f;

		if(betweenRigidbodies==true)
			responseForce/=contacts.size();



		bodyA->GetWorld()->gizmos.push_back(new QGizmoCircle(contact->position,2.0f));



		QBody *referenceBody=contact->referenceParticles[0]->GetOwnerMesh()->GetOwnerBody();
		QBody *incidentBody=contact->particle->GetOwnerMesh()->GetOwnerBody();



		QRigidBody *refRigidBody=nullptr;
		QRigidBody *incRigidBody=nullptr;

		if(referenceBody->GetBodyType()==QBody::BodyTypes::RIGID)
			refRigidBody=static_cast<QRigidBody*>(referenceBody);
		if(incidentBody->GetBodyType()==QBody::BodyTypes::RIGID)
			incRigidBody=static_cast<QRigidBody*>(incidentBody);

		QVector rRef=contact->position-referenceBody->GetPosition();
		QVector rInc=contact->position-incidentBody->GetPosition();

		if(i==0){
			linearRelativeVelocity=GetRelativeVelocity(contact->particle,contact->referenceParticles,rRef,rInc);
		}







		bool cancelSolving=false;

		//Checking Area Bodies

		if(referenceBody->GetBodyType()==QBody::BodyTypes::AREA){
			QAreaBody *refAreaBody=static_cast<QAreaBody*>(referenceBody);
			refAreaBody->AddCollidedBody(incidentBody);
			cancelSolving=true;
		}
	
		if(incidentBody->GetBodyType()==QBody::BodyTypes::AREA){
			QAreaBody *incAreaBody=static_cast<QAreaBody*>(incidentBody);
			incAreaBody->AddCollidedBody(referenceBody);
			cancelSolving=true;
		}

		//Calling Events

		QBody::CollisionInfo colInfoRef(contact->position,incidentBody,-contact->normal,contact->penetration);
		bool collisionEnabledByRef=referenceBody->OnCollision(colInfoRef);
		if(referenceBody->CollisionEventListener!=nullptr){
			bool listenerResult=referenceBody->CollisionEventListener(referenceBody,colInfoRef);
			if(collisionEnabledByRef==true)
				collisionEnabledByRef=listenerResult;
		}



		QBody::CollisionInfo colInfoInc(contact->position,referenceBody,contact->normal,contact->penetration);
		bool collisionEnabledByInc=incidentBody->OnCollision(colInfoInc);
		if(incidentBody->CollisionEventListener!=nullptr){
			bool listenerResult=incidentBody->CollisionEventListener(incidentBody,colInfoInc);
			if(collisionEnabledByInc==true)
				collisionEnabledByInc=listenerResult;
		}

		


		if(collisionEnabledByRef==false || collisionEnabledByInc==false){
			cancelSolving=true;
		}

		if(cancelSolving==true)
			continue;



		//APPLYING FORCES TO BODIES
		QVector refResponseForce;
		QVector incResponseForce;


		refResponseForce=-responseForce;
		incResponseForce=responseForce;


		if(isCollisionOneSide==false){
			refResponseForce*=incidentBody->GetMass()*invMass;
			incResponseForce*=referenceBody->GetMass()*invMass;
		}

		if(incidentBody->CanGiveCollisionResponseTo(referenceBody)){
			contact->solved=true;
			if(refRigidBody!=nullptr){
				//For RigidBodies
				refRigidBody->ApplyForce(refResponseForce,rRef,true);
			}else{
				//For Softbodies
				if(contact->referenceParticles.size()==2){
					QParticle::ApplyForceToParticleSegment(contact->referenceParticles[0],contact->referenceParticles[1],refResponseForce,contact->position );
				}else{
					contact->referenceParticles[0]->ApplyForce(refResponseForce);
				}
			}
		}

		if(referenceBody->CanGiveCollisionResponseTo(incidentBody)){
			contact->solved=true;
			if(incRigidBody!=nullptr){
				//For RigidBodies
				incRigidBody->ApplyForce(incResponseForce,rInc,true);
			}else{
				//For Softbodies
				contact->particle->ApplyForce(incResponseForce);
			}
		}



	}




}

void QManifold::SolveFrictionAndVelocities()
{
	//Don't apply friction and velocity to kinematic and static body pairs.
	bool isBodyADynamic=bodyA->isKinematic==false && bodyA->mode!=QBody::STATIC;
	bool isBodyBDynamic=bodyB->isKinematic==false && bodyB->mode!=QBody::STATIC;
	if(!isBodyADynamic && !isBodyBDynamic)
		return;


	for(int i=0;i<contacts.size();i++){
		QCollision::Contact *contact=&contacts[i];
		//Don't apply friction and velocity to not solved contacts
		if(contact->solved==false) continue;


		QBody *referenceBody=contact->referenceParticles[0]->GetOwnerMesh()->GetOwnerBody();
		QBody *incidentBody=contact->particle->GetOwnerMesh()->GetOwnerBody();


		QRigidBody *refRigidBody=nullptr;
		QRigidBody *incRigidBody=nullptr;

		if(referenceBody->GetBodyType()==QBody::BodyTypes::RIGID)
			refRigidBody=static_cast<QRigidBody*>(referenceBody);
		if(incidentBody->GetBodyType()==QBody::BodyTypes::RIGID)
			incRigidBody=static_cast<QRigidBody*>(incidentBody);

		QVector rRef=contact->position-referenceBody->GetPosition();
		QVector rInc=contact->position-incidentBody->GetPosition();


		//Velocity Correction
		QVector relVel;

		if(i==0 && restitution>0.0f){
			auto j=linearRelativeVelocity.Dot(contact->normal);//+contact->prevImpulseScalar;
			if(j>restitution*2.0f){
				relVel=GetRelativeVelocity(contact->particle,contact->referenceParticles,QVector::Zero(),QVector::Zero());
				QVector tangent=relVel-(relVel.Dot(contact->normal)*contact->normal );
				auto jn=(j*contact->normal)*restitution-tangent;


				auto refImpulse=-jn;
				auto incImpulse=jn;

				if(isCollisionOneSide==false){
					refImpulse*=incidentBody->GetMass()*invMass;
					incImpulse*=referenceBody->GetMass()*invMass;
				}


				if(refRigidBody!=nullptr){
					//For RigidBodies
					if(incidentBody->CanGiveCollisionResponseTo(referenceBody) && referenceBody->isKinematic==false){
						refRigidBody->SetPreviousPosition(referenceBody->GetPosition()-refImpulse);
					}
				}else{
					//For Softbodies
				}

				if(incRigidBody!=nullptr){
					//For RigidBodies
					if(referenceBody->CanGiveCollisionResponseTo(incidentBody) && incidentBody->isKinematic==false){
						incRigidBody->SetPreviousPosition(incidentBody->GetPosition()-incImpulse);
					}
				}else{
					//For Softbodies
				}

			}
		}

		//FRICTIONS


		relVel=GetRelativeVelocity(contact->particle,contact->referenceParticles,rRef,rInc);
		QVector frictionForce=QBody::ComputeFriction(incidentBody,referenceBody,contact->normal,contact->penetration,relVel);

		QVector refResponseForce;
		QVector incResponseForce;


		//APPLYING FRICTION FORCE TO BODIES

		refResponseForce=frictionForce;
		incResponseForce=-frictionForce;

		if(isCollisionOneSide==false){
			refResponseForce*=incidentBody->GetMass()*invMass;
			incResponseForce*=referenceBody->GetMass()*invMass;
		}


		if(incidentBody->CanGiveCollisionResponseTo(referenceBody) && referenceBody->isKinematic!=true){
			if(refRigidBody!=nullptr){
				//For RigidBodies
				refRigidBody->ApplyForce(refResponseForce,rRef);

			}else{
				//For Softbodies
				if(contact->referenceParticles.size()==2){
					QParticle::ApplyForceToParticleSegment(contact->referenceParticles[0],contact->referenceParticles[1],refResponseForce,contact->particle->GetGlobalPosition());
				}else{
					contact->referenceParticles[0]->ApplyForce(refResponseForce);
				}
			}
		}

		if(referenceBody->CanGiveCollisionResponseTo(incidentBody) && incidentBody->isKinematic!=true){
			if(incRigidBody!=nullptr){
				//For RigidBodies
				incRigidBody->ApplyForce(incResponseForce,rInc);

			}else{
				//For Softbodies
				contact->particle->ApplyForce(incResponseForce);
			}
		}






	}

}








