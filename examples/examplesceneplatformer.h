#ifndef EXAMPLESCENEPLATFORMER_H
#define EXAMPLESCENEPLATFORMER_H
#include "../qexamplescene.h"
#include "../QuarkPhysics/qareabody.h"

class ExampleScenePlatformer : public QExampleScene
{

public:
	ExampleScenePlatformer(QVector sceneSize);
	void OnPlayerPreStep(QBody *body);
	void OnPlayerStep(QBody *body);
	void OnPlayerCollision(QBody *body,QBody::CollisionInfo info);
	void OnPlatformPreStep(QBody *body);
	void OnCointCollisionEnter(QAreaBody *body,QBody *collidedBody);

	void OnMouseMoved(QVector mousePosition);

	//Player properties
	QRigidBody *player;
	QVector velocity;
	QBody *currentFloor=nullptr;
	int jumpTickCount=30;
	int jumpTickDown=0;

	//Side checkers
	bool isOnFloor=false;
	bool isOnCeiling=false;
	bool isOnLeftWall=false;
	bool isOnRightWall=false;

	//Movable Platform	
	QRigidBody *mBlock;
	QVector mBlockStartPos=QVector::Zero();
	QVector mBlockEndPos=QVector::Zero();
	QVector mBlockTargetPos=QVector::Zero();
	float mBlockMoveSpeed=1.0f;
};

#endif // EXAMPLESCENEPLATFORMER_H
