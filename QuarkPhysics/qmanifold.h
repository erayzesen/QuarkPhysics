#ifndef QMANIFOLD_H
#define QMANIFOLD_H
#include "qbody.h"
#include "qcollision.h"
/** 
 * @brief QManifold retrieves collision data from collision tests between two QBody objects using QCollision methods and resolves collisions based on this data. The Solve() method applies collision reactions by changing the positions of contact partners, while the SolveFrictionAndVelocities() method applies friction to the contact partners and adjusts their velocity values.
 */
class QManifold
{
	QVector GetRelativeVelocity(QParticle *contactParticle,vector<QParticle*> referenceParticles, QVector rRef,QVector rInc);
	QVector linearRelativeVelocity=QVector::Zero();

	//One time calculate properties
	float restitution=0.0f;
	float invMass;
	bool isCollisionOneSide=false;
public:


	/**
	 * Creates new manifold with two bodies.
	 * @param bodyA A body in the world.
	 * @param bodyB Another body in the world. 
	 */
	QManifold(QBody *bodyA,QBody *bodyB);

	QBody *bodyA;
	QBody *bodyB;

	/** The collection of contacs from collision test using QCollision methods. */
	vector<QCollision::Contact> contacts;

	/** Applies collision reactions by changing the positions of the contact partners. */
	void Solve();

	/** Applies friction to the contact partners and adjust their velocity values */
	void SolveFrictionAndVelocities();


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
