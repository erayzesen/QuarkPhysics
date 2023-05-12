
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

#include "examplescenemixedbodies.h"
#include <cmath>
#include <cstdlib>
#include <vector>


ExampleSceneMixedBodies::ExampleSceneMixedBodies(QVector sceneSize):QExampleScene(sceneSize)
{
	//world->SetEnableSleeping(false);

	// Floor and walls
	QBody *floor=new QBody();
	floor->AddMesh(QMesh::CreateWithRect(QVector(960,64),QVector(0.0f,0.0f) ) )->SetPosition(QVector(512.0f,550.0f));
	floor->SetMode(QBody::Modes::STATIC);
	world->AddBody(floor);


	QBody *wallLeft=new QBody();
	wallLeft->AddMesh(QMesh::CreateWithRect(QVector(64,1500),QVector(0.0f,0.0f) ) )->SetPosition(floor->GetPosition()-QVector(960*0.5f-32.0f,1500*0.5f+32.0f));
	wallLeft->SetMode(QBody::Modes::STATIC);
	world->AddBody(wallLeft);

	QBody *wallRight=new QBody();
	wallRight->AddMesh(QMesh::CreateWithRect(QVector(64,1500),QVector(0.0f,0.0f) ) )->SetPosition(floor->GetPosition()+QVector(960*0.5f-32.0f,-1500*0.5f-32.0f));
	wallRight->SetMode(QBody::Modes::STATIC);
	world->AddBody(wallRight);

	//Determines to adding body objects to a specific position.
	QVector startPosition=QVector(128,100);
	QVector positionPointer=startPosition;

	//Adding random privitive objects
	for (int n=0;n<3;n++){
		positionPointer=startPosition-QVector(0,n*64);
		for (int i=0; i<7; i++) {
			QRigidBody *nBody=new QRigidBody();
			QMesh *nMesh;
			int rndPrivimiteType=rand() % 2;
			int rndRadius=16+(rand() %32);
			if(rndPrivimiteType==0){
				nMesh=QMesh::CreateWithCircle(rndRadius);
			}else{
				int rndSideCount=3+(rand()%8);
				nMesh=QMesh::CreateWithPolygon(rndRadius, rndSideCount);
			}
			positionPointer+=QVector(rndRadius*2+32.0f,0);
			nBody->AddMesh(nMesh)->SetPosition(positionPointer);
			world->AddBody(nBody);
		
		}
	}


	//Adding word shaped meshes from files
	QSoftBody *wordBody;
	float rigidity=0.3f;
	float matchingRate=0.35f;
	positionPointer=QVector(180,425);
	QVector diff=QVector(170,0.0);
	wordBody=new QSoftBody();
	wordBody->SetPosition(positionPointer);
	wordBody->AddMeshesFromFile("../resources/mesh_files/word_q.qmesh");
	wordBody->SetRigidity(rigidity)->SetShapeMatchingEnabled(true)->SetShapeMatchingRate(matchingRate);
	wordBody->SetSelfCollisionsEnabled(true);
	world->AddBody(wordBody);

	positionPointer+=diff;

	wordBody=new QSoftBody();
	wordBody->SetPosition(positionPointer );
	wordBody->AddMeshesFromFile("../resources/mesh_files/word_u.qmesh");
	wordBody->SetRigidity(rigidity)->SetShapeMatchingEnabled(true)->SetShapeMatchingRate(matchingRate);
	wordBody->SetSelfCollisionsEnabled(true);
	world->AddBody(wordBody);


	positionPointer+=diff;

	wordBody=new QSoftBody();
	wordBody->SetPosition(positionPointer);
	wordBody->AddMeshesFromFile("../resources/mesh_files/word_a.qmesh");
	wordBody->SetRigidity(rigidity)->SetShapeMatchingEnabled(true)->SetShapeMatchingRate(matchingRate);
	wordBody->SetSelfCollisionsEnabled(true);
	world->AddBody(wordBody);


	positionPointer+=diff;

	wordBody=new QSoftBody();
	wordBody->SetPosition(positionPointer);
	wordBody->AddMeshesFromFile("../resources/mesh_files/word_r.qmesh");
	wordBody->SetRigidity(rigidity)->SetShapeMatchingEnabled(true)->SetShapeMatchingRate(matchingRate);
	wordBody->SetSelfCollisionsEnabled(true);
	world->AddBody(wordBody);


	positionPointer+=diff;

	wordBody=new QSoftBody();
	wordBody->SetPosition(positionPointer);
	wordBody->AddMeshesFromFile("../resources/mesh_files/word_k.qmesh");
	wordBody->SetRigidity(rigidity)->SetShapeMatchingEnabled(true)->SetShapeMatchingRate(matchingRate);
	wordBody->SetSelfCollisionsEnabled(true);
	world->AddBody(wordBody);


}
void ExampleSceneMixedBodies::OnMousePressed(QVector mousePosition)
{
//	auto hitBodies=world->GetBodiesHitByPoint(mousePosition,1);
//	if(hitBodies.size()>0){
//		world->RemoveBody(hitBodies[0]);
//	}
	QExampleScene::OnMousePressed(mousePosition);
//	auto hitParticles=world->GetParticlesCloseToPoint(mousePosition,8.0f,1,false);
//	if(hitParticles.size()>0){
//		cout<<"bulundu"<<endl;
//		QParticle *particle=hitParticles[0];
//		QMesh *ownerMesh=particle->GetOwnerMesh();
//		ownerMesh->RemoveParticle(particle);
//	}
	//AddBlobBody(mousePosition.x,mousePosition.y,64.0f,10);
}
