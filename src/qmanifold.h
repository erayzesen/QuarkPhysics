#ifndef QMANIFOLD_H
#define QMANIFOLD_H
#include "qbody.h"
#include "qcollision.h"

class QManifold
{
	QVector GetRelativeVelocity(QParticle *contactParticle,vector<QParticle*> referenceParticles, QVector rRef,QVector rInc);
public:
	QManifold(QBody *bodyA,QBody *bodyB);

	QBody *bodyA;
	QBody *bodyB;

	vector<QCollision::Contact> contacts;

	//void Update(vector<QCollision::Contact> newContacts);
	void Solve();

	void SolveFrictionAndVelocities();



	QVector linearRelativeVelocity=QVector::Zero();

	//One time calculate properties
	float restitution=0.0f;
	float invMass;
	bool isCollisionOneSide=false;





};

struct QManifoldKey{
public:
	QManifoldKey(QBody *bA,QBody *bB){
		if(bA<bB){
			bodyA=bA;
			bodyB=bB;
		}else{
			bodyA=bB;
			bodyB=bA;
		}
	}
	QBody *bodyA;
	QBody *bodyB;
};

inline bool operator <(const QManifoldKey& mk1,const QManifoldKey& mk2){
	if(mk1.bodyA<mk2.bodyA){
		return true;
	}
	if(mk1.bodyA==mk2.bodyA && mk1.bodyB<mk2.bodyB){
		return true;
	}
	return false;
}

#endif // QMANIFOLD_H
