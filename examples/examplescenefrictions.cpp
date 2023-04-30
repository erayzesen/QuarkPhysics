
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

#include "examplescenefrictions.h"


ExampleSceneFrictions::ExampleSceneFrictions(QVector sceneSize):QExampleScene(sceneSize){
	world->SetEnableSleeping(false);
	//Adding Floor
	QRigidBody *floor=new QRigidBody();
	floor->AddMesh(QMesh::CreateWithRect(QVector(3000,64),QVector(0.0f,0.0f) ) )->SetPosition(QVector(512.0f,550.0f));
	floor->SetMode(QBody::Modes::STATIC);
	floor->SetFriction(0.5f)->SetStaticFriction(1.0f);
	world->AddBody(floor);

	//Adding platforms and bodies
	QRigidBody* platform;
	QRigidBody* ball;
	QRigidBody* box;

	QVector platformPos(200,176);
	QVector margin(0,120);
	float rotationDeg=6.2f;

	//01

	platform=new QRigidBody();
	platform->AddMesh( QMesh::CreateWithRect(QVector(600,32) ))->SetMode(QBody::Modes::STATIC);
	platform->SetPosition(platformPos)->SetRotationDegree(6.2f);
	world->AddBody(platform);

	box=new QRigidBody();
	box->AddMesh(QMesh::CreateWithRect(QVector(32,32) ));
	box->SetPosition(platform->GetPosition()-QVector(0.0f,41) )->SetRotationDegree(6.2f);
	box->SetFriction(0.01f);
	world->AddBody(box);

	ball=new QRigidBody();
	ball->AddMesh(QMesh::CreateWithCircle( 16.0f ));
	ball->SetPosition(platform->GetPosition()-QVector(130,51) )->SetRotationDegree(6.2f);
	ball->SetFriction(0.1f)->SetStaticFriction(0.0f);
	world->AddBody(ball);


	//02
	platformPos+=margin;
	platform=new QRigidBody();
	platform->AddMesh( QMesh::CreateWithRect(QVector(600,32) ))->SetMode(QBody::Modes::STATIC);
	platform->SetPosition(platformPos)->SetRotationDegree(6.2f);
	world->AddBody(platform);

	box=new QRigidBody();
	box->AddMesh(QMesh::CreateWithRect(QVector(32,32) ));
	box->SetPosition(platform->GetPosition()-QVector(0.0f,41) )->SetRotationDegree(6.2f);
	box->SetFriction(0.05f);
	world->AddBody(box);

	ball=new QRigidBody();
	ball->AddMesh(QMesh::CreateWithCircle( 16.0f ));
	ball->SetPosition(platform->GetPosition()-QVector(130,51) )->SetRotationDegree(6.2f);
	ball->SetFriction(0.2f)->SetStaticFriction(0.0f);
	world->AddBody(ball);


	//03
	platformPos+=margin;
	platform=new QRigidBody();
	platform->AddMesh( QMesh::CreateWithRect(QVector(600,32) ))->SetMode(QBody::Modes::STATIC);
	platform->SetPosition(platformPos)->SetRotationDegree(6.2f);
	world->AddBody(platform);

	box=new QRigidBody();
	box->AddMesh(QMesh::CreateWithRect(QVector(32,32) ));
	box->SetPosition(platform->GetPosition()-QVector(0.0f,41) )->SetRotationDegree(6.2f);
	box->SetFriction(0.05f)->SetStaticFriction(0.01f);
	world->AddBody(box);

	//Adding Box Stack
	QVector startPos=floor->GetPosition()+QVector(100,-48.0f);
	for(int i=0;i<10;i++){
		QVector pos=startPos-(i*QVector(0,32));
		box=new QRigidBody();
		box->AddMesh(QMesh::CreateWithRect(QVector(32,32) ))->SetPosition(pos);
		world->AddBody(box);
	}


}
