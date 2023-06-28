#ifndef QJOINT_H
#define QJOINT_H
#include "QRigidBody.h"

class QJoint
{
	//Base Properties

	QRigidBody * bodyA=nullptr;
	QRigidBody * bodyB=nullptr;

	QVector anchorA;
	QVector anchorB;

	QVector anchorGlobalA;
	QVector anchorGlobalB;

	bool collisionsEnabled=false;

	bool enabled=true;

	float rigidity=1.0f;

	//Addinational Properties
	float distance=0.0f; //it makes distance joint
	bool grooveEnabled=false; // it makes groove

	QWorld *world;
public:
	QJoint(QRigidBody *bodyA,QVector anchorWorldPositionA,QVector anchorWorldPositionB,QRigidBody* bodyB=nullptr); // There are defined two achor points. Default distance value is the distance between the two points.
	QJoint(QRigidBody *bodyA,QVector commonAnchorWorldPosition,QRigidBody* bodyB=nullptr) : QJoint(bodyA,commonAnchorWorldPosition,commonAnchorWorldPosition,bodyB){}; // There is a defined common anchor position, default distance is the zero.
	~QJoint();

	//Get Methods
	inline QRigidBody * GetBodyA(){
		return bodyA;
	}
	inline QRigidBody * GetBodyB(){
		return bodyB;
	}
	inline QVector GetAnchorAPosition(){
		return anchorA;
	}
	inline QVector GetAnchorBPosition(){
		return anchorB;
	}
	inline QVector GetAnchorAGlobalPosition(){
		return anchorGlobalA;
	}
	inline QVector GetAnchorBGlobalPosition(){
		return anchorGlobalB;
	}
	inline bool GetCollisionEnabled(){
		return collisionsEnabled;
	}
	inline float GetRigidity(){
		return rigidity;
	}
	inline float GetDistance(){
		return distance;
	}
	inline bool GetGrooveEnabled(){
		return grooveEnabled;
	}

	//Set Methods (It returns joint)
	inline QJoint * SetBodyA(QRigidBody *body){
		bodyA=body;
		return this;
	}
	inline QJoint * SetBodyB(QRigidBody *body){
		bodyB=body;
		return this;
	}
	inline QJoint * SetAnchorAPosition(QVector worldPosition){
		if(bodyA!=nullptr){
			this->anchorA=(worldPosition-bodyA->GetPosition()).Rotated(-bodyA->GetRotation());
		}else{
			this->anchorA=worldPosition;
		}
		return this;
	}

	inline QJoint * SetAnchorBPosition(QVector worldPosition){
		if(bodyB!=nullptr){
			this->anchorB=(worldPosition-bodyB->GetPosition()).Rotated(-bodyB->GetRotation());
		}else{
			this->anchorB=worldPosition;
		}
		return this;
	}

	inline QJoint* SetRigidity(float value){
		rigidity=value;
		return this;
	}
	inline QJoint* SetDistance(float value){
		distance=value;
		return this;
	}

	inline QJoint* SetGrooveEnabled(bool value){
		grooveEnabled=value;
		return this;
	}

	QJoint* SetCollisionEnabled(bool value);




	virtual void Update();

	friend class QWorld;




};



#endif // QJOINT_H
