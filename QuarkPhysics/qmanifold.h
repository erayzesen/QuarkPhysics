
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
	vector<QCollision::Contact*> contacts;

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
