#ifndef QRIGIDBODY_H
#define QRIGIDBODY_H
#include "qbody.h"


class QRigidBody : public QBody
{
	bool fixedRotation=false;
public:
	QRigidBody();

	//Get Methods
	bool GetFixedRotation(){
		return fixedRotation;
	}

	//Set Methods
	QRigidBody* SetFixedRotation(bool value){
		fixedRotation=value;
		return this;
	}




	//#Rigidbody Methods
	QRigidBody* ApplyForce(QVector force,QVector r,bool updateMeshTransforms=true);
	QRigidBody* ApplyImpulse(QVector impulse,QVector r);
	void Update();


};

#endif // QRIGIDBODY_H
