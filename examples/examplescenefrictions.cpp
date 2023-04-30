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
