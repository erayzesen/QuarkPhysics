
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

#include "examplescenejoints.h"

ExampleSceneJoints::ExampleSceneJoints(QVector sceneSize):QExampleScene(sceneSize)
{
	//world->gravity=QVector::Zero();
	world->SetEnableSleeping(false);
	PinJointSample(QVector(200,200) );
	SpringDistanceJointSample(QVector(400,200) );
	GrooveJointSample(QVector(600,200) );
	DistanceJointSample(QVector(800,200) );
}

void ExampleSceneJoints::PinJointSample(QVector pos)
{
	QVector sPos=pos;
	float df=4; //distance factor
	sPos+=QVector(0,spawnCircleRadius);
	QRigidBody *ball=AddCircleBody(sPos.x,sPos.y);
	sPos+=QVector(0,spawnRectSize*0.5f+spawnCircleRadius);
	QRigidBody *box=AddRectBody(sPos.x,sPos.y);
	sPos+=QVector(0,spawnRectSize*0.5f+spawnPolygonRadius);
	QRigidBody *polygon=AddPolygonBody(sPos.x,sPos.y,6);


	QJoint *jointToAir=new QJoint(ball,ball->GetPosition()-QVector(0,spawnCircleRadius),nullptr);
	world->AddJoint(jointToAir);

	QJoint *jointBallToBox=new QJoint(ball,ball->GetPosition()+QVector(0,spawnCircleRadius),box->GetPosition()-QVector(0,spawnRectSize*0.5f),box);
	world->AddJoint(jointBallToBox);

	QJoint *jointBoxToPolygon=new QJoint(box,box->GetPosition()+QVector(0,spawnRectSize*0.5f),polygon->GetPosition()-QVector(0,spawnPolygonRadius),polygon);
	world->AddJoint(jointBoxToPolygon);


}

void ExampleSceneJoints::DistanceJointSample(QVector pos)
{
	QVector sPos=pos;
	float df=4; //distance factor
	int boxCount=6;
	QVector boxSize(16,48);
	float distance=spawnRectSize;
	vector<QRigidBody*> chainBodies;
	for(int i=0;i<boxCount;i++){
		QRigidBody *box=new QRigidBody();
		box->AddMesh(QMesh::CreateWithRect(boxSize) )->SetPosition(sPos);
		world->AddBody(box);
		chainBodies.push_back(box);
		sPos+=QVector(0,distance+spawnRectSize);
	}
	QVector anchorDelta(0,boxSize.y*0.5-boxSize.x*0.5f);
	for(int i=0;i<chainBodies.size()-1;i++){
		QRigidBody *b=chainBodies[i];
		QRigidBody *nb=chainBodies[i+1];
		QJoint *joint=new QJoint(b,b->GetPosition()+anchorDelta,nb->GetPosition()-anchorDelta,nb);
		world->AddJoint(joint);
		if(i==0){
			QJoint *pin=new QJoint(b,b->GetPosition()-anchorDelta,nullptr);
			world->AddJoint(pin);
		}
	}
}

void ExampleSceneJoints::SpringDistanceJointSample(QVector pos)
{
	QVector sPos=pos;
	float df=4; //distance factor
	int ballCount=6;
	float distance=spawnCircleRadius;
	vector<QRigidBody*> chainBodies;
	for(int i=0;i<ballCount;i++){
		QRigidBody *ball=AddCircleBody(sPos.x,sPos.y);
		chainBodies.push_back(ball);
		sPos+=QVector(0,distance+spawnCircleRadius);
	}
	for(int i=0;i<chainBodies.size()-1;i++){
		QRigidBody *b=chainBodies[i];
		QRigidBody *nb=chainBodies[i+1];
		QJoint *joint=new QJoint(b,b->GetPosition(),nb->GetPosition(),nb);
		joint->SetRigidity(0.1f);
		world->AddJoint(joint);
		if(i==0){
			QJoint *pin=new QJoint(b,b->GetPosition(),nullptr);
			world->AddJoint(pin);
		}
	}
}

void ExampleSceneJoints::GrooveJointSample(QVector pos)
{
	QVector sPos=pos;
	float df=4; //distance factor
	sPos+=QVector(0,spawnCircleRadius);
	QRigidBody *boxA=AddRectBody(sPos.x,sPos.y);
	sPos+=QVector(0,spawnRectSize*1);
	QRigidBody *boxB=AddRectBody(sPos.x,sPos.y);



	QJoint *jointToAir=new QJoint(boxA,boxA->GetPosition()-QVector(0,spawnRectSize*0.5f),nullptr);
	world->AddJoint(jointToAir);

	QJoint *jointBoxA2BoxB=new QJoint(boxA,boxA->GetPosition()+QVector(0,spawnRectSize*0.5f),boxB->GetPosition()-QVector(0,spawnRectSize*0.5f),boxB);
	jointBoxA2BoxB->SetGrooveEnabled(true);
	jointBoxA2BoxB->SetLength(spawnRectSize*3.0f);
	world->AddJoint(jointBoxA2BoxB);


}

