#include "qmanifold.h"
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

	if(bodyA->GetMode()==QBody::STATIC || bodyB->GetMode()==QBody::STATIC){
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

	if( bodyInc->GetSimulationModel()==QBody::RIGID_BODY  ){
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
	if(bodyA->GetSimulationModel()==QBody::SimulationModels::RIGID_BODY && bodyB->GetSimulationModel()==QBody::SimulationModels::RIGID_BODY)
		betweenRigidbodies=true;



	for(int i=0;i<contacts.size();i++){
		QCollision::Contact *contact=&contacts[i];

		if(betweenRigidbodies){
			contact->penetration*=0.75f;
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

		if(typeid(*referenceBody)==typeid(QRigidBody))
			refRigidBody=dynamic_cast<QRigidBody*>(referenceBody);
		if(typeid(*incidentBody)==typeid(QRigidBody))
			incRigidBody=dynamic_cast<QRigidBody*>(incidentBody);

		QVector rRef=contact->position-referenceBody->GetPosition();
		QVector rInc=contact->position-incidentBody->GetPosition();

		if(i==0){
			linearRelativeVelocity=GetRelativeVelocity(contact->particle,contact->referenceParticles,rRef,rInc);
		}






		QVector refResponseForce;
		QVector incResponseForce;




		//APPLYING FORCES TO BODIES
		refResponseForce=-responseForce;
		incResponseForce=responseForce;




		if(isCollisionOneSide==false){
			refResponseForce*=incidentBody->GetMass()*invMass;
			incResponseForce*=referenceBody->GetMass()*invMass;
		}

		if(incidentBody->CanGiveCollisionResponseTo(referenceBody)){
			if(refRigidBody!=nullptr){
				//For RigidBodies
				refRigidBody->ApplyForce(refResponseForce,rRef,false);
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
			if(incRigidBody!=nullptr){
				//For RigidBodies
				incRigidBody->ApplyForce(incResponseForce,rInc,false);
			}else{
				//For Softbodies
				contact->particle->ApplyForce(incResponseForce);
			}
		}





	}



}

void QManifold::SolveFrictionAndVelocities()
{


	for(int i=0;i<contacts.size();i++){
		QCollision::Contact *contact=&contacts[i];


		QBody *referenceBody=contact->referenceParticles[0]->GetOwnerMesh()->GetOwnerBody();
		QBody *incidentBody=contact->particle->GetOwnerMesh()->GetOwnerBody();


		QRigidBody *refRigidBody=nullptr;
		QRigidBody *incRigidBody=nullptr;

		if(typeid(*referenceBody)==typeid(QRigidBody))
			refRigidBody=dynamic_cast<QRigidBody*>(referenceBody);
		if(typeid(*incidentBody)==typeid(QRigidBody))
			incRigidBody=dynamic_cast<QRigidBody*>(incidentBody);

		QVector rRef=contact->position-referenceBody->GetPosition();
		QVector rInc=contact->position-incidentBody->GetPosition();


		//Velocity Correction
		QVector relVel;

		//linearRelativeVelocity=GetRelativeVelocity(referenceBody,incidentBody,rRef,rInc);
		if(i==0 && restitution>0.0f){
			auto j=linearRelativeVelocity.Dot(contact->normal);//+contact->prevImpulseScalar;
			//contact->prevImpulseScalar=j;
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
					if(incidentBody->CanGiveCollisionResponseTo(referenceBody)){
						refRigidBody->SetPreviousPosition(referenceBody->GetPosition()-refImpulse);
					}
				}else{
					//For Softbodies
				}

				if(incRigidBody!=nullptr){
					//For RigidBodies
					if(referenceBody->CanGiveCollisionResponseTo(incidentBody)){
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


		//APPLYING FRICTION IMPULSE FORCE TO BODIES

		refResponseForce=frictionForce;
		incResponseForce=-frictionForce;

		if(isCollisionOneSide==false){
			refResponseForce*=incidentBody->GetMass()*invMass;
			incResponseForce*=referenceBody->GetMass()*invMass;
		}


		if(incidentBody->CanGiveCollisionResponseTo(referenceBody)){
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

		if(referenceBody->CanGiveCollisionResponseTo(incidentBody)){
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








