
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
