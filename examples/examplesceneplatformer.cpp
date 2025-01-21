
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

#include "examplesceneplatformer.h"
#include <functional>
#include "../QuarkPhysics/qareabody.h"
using namespace std;
ExampleScenePlatformer::ExampleScenePlatformer(QVector sceneSize):QExampleScene(sceneSize)
{
	//Creating platforms and player
	//
	world->SetSleepingEnabled(false);


	QVector playerStartPosition=QVector(303,450);

	//Adding Player
	player=new QPlatformerBody();
	player->AddMesh( QMesh::CreateWithRect(QVector(32,32) ) )->SetPosition(playerStartPosition );
	world->AddBody(player);
	player->StepEventListener=std::bind(&ExampleScenePlatformer::OnPlayerStep,this,std::placeholders::_1);

	//ADDING FLOORS
	QRigidBody *floor;

	//Creating Slopped Custom Platforms
	floor=new QRigidBody();
	QMesh::MeshData customFloorMeshData;
	customFloorMeshData.particlePositions={QVector(203,480),QVector(380,480),QVector(500,410), QVector(690,410),QVector(750,480),QVector(900,480) };
	size_t topPointsCount=customFloorMeshData.particlePositions.size();
	for(size_t i=topPointsCount;i>0;--i ){
		QVector point=customFloorMeshData.particlePositions[i-1];
		point.y=550;
		customFloorMeshData.particlePositions.push_back( point );
	}
	for(size_t i=0;i<customFloorMeshData.particlePositions.size();++i ){
		customFloorMeshData.polygon.push_back(i);
		customFloorMeshData.particleInternalValues.push_back(false);
		customFloorMeshData.particleRadValues.push_back(0.5f);
		customFloorMeshData.particleLazyValues.push_back(false);
		customFloorMeshData.particleEnabledValues.push_back(true);
	}
	
	floor->AddMesh(QMesh::CreateWithMeshData(customFloorMeshData,false,true) );
	floor->SetMode(QBody::Modes::STATIC);
	world->AddBody(floor);


	//Adding Rectangle Platforms
	floor=new QRigidBody();
	floor->AddMesh(QMesh::CreateWithRect(QVector(128,64) ) )->SetPosition(QVector(99.0f,400.0f));
	floor->SetMode(QBody::Modes::STATIC);
	world->AddBody(floor);

	
	QRigidBody *wall;
	wall=new QRigidBody();
	wall->AddMesh(QMesh::CreateWithRect(QVector(64,300) ) )->SetPosition(QVector(960.0f,270.0f));
	wall->SetMode(QBody::Modes::STATIC);
	world->AddBody(wall);

	wall=new QRigidBody();
	wall->AddMesh(QMesh::CreateWithRect(QVector(64,200) ) )->SetPosition(QVector(810.0f,275.0f));
	wall->SetMode(QBody::Modes::STATIC);
	world->AddBody(wall);

	//Adding a Moveable Platform
	mBlock=new QRigidBody();
	mBlock->AddMesh(QMesh::CreateWithRect( QVector(128,32)) )->SetPosition(QVector(630.0f,190.0f));
	mBlock->SetKinematicEnabled(true)->SetFixedRotationEnabled(true);
	mBlockStartPos=mBlock->GetPosition();
	mBlockEndPos=mBlockStartPos+QVector(-320,150);
	mBlockTargetPos=mBlockEndPos;
	world->AddBody(mBlock);
	mBlock->StepEventListener=std::bind(&ExampleScenePlatformer::OnPlatformStep,this,std::placeholders::_1);

	

	

	//Adding Coints(Using QAreaBody)
	QAreaBody *coint;
	coint=new QAreaBody();
	coint->AddMesh( QMesh::CreateWithCircle(8.0f)  )->SetPosition(QVector(570,320) );
	coint->CollisionEnterEventListener=bind(&ExampleScenePlatformer::OnCointCollisionEnter,this,placeholders::_1,placeholders::_2);
	world->AddBody(coint);

	coint=new QAreaBody();
	coint->AddMesh( QMesh::CreateWithCircle(8.0f)  )->SetPosition(QVector(606,320) );
	coint->CollisionEnterEventListener=bind(&ExampleScenePlatformer::OnCointCollisionEnter,this,placeholders::_1,placeholders::_2);
	world->AddBody(coint);
	
	coint=new QAreaBody();
	coint->AddMesh( QMesh::CreateWithCircle(8.0f)  )->SetPosition(QVector(640,320) );
	coint->CollisionEnterEventListener=bind(&ExampleScenePlatformer::OnCointCollisionEnter,this,placeholders::_1,placeholders::_2);
	world->AddBody(coint);


	coint=new QAreaBody();
	coint->AddMesh( QMesh::CreateWithCircle(8.0f)  )->SetPosition(QVector(84,350) );
	coint->CollisionEnterEventListener=bind(&ExampleScenePlatformer::OnCointCollisionEnter,this,placeholders::_1,placeholders::_2);
	world->AddBody(coint);

	coint=new QAreaBody();
	coint->AddMesh( QMesh::CreateWithCircle(8.0f)  )->SetPosition(QVector(120,350) );
	coint->CollisionEnterEventListener=bind(&ExampleScenePlatformer::OnCointCollisionEnter,this,placeholders::_1,placeholders::_2);
	world->AddBody(coint);

}



void ExampleScenePlatformer::OnPlayerStep(QBody *body)
{
	
	//Player Movement Properties

	//Walk
	//Defining walk side according to keyboard input
	int walkSide=(sf::Keyboard::isKeyPressed(sf::Keyboard::Right) )-(sf::Keyboard::isKeyPressed(sf::Keyboard::Left) );
	player->Walk( walkSide);

	//Wall Mode
	bool wallMode=false;
	float wallOffset=3.0f;
	int wallSide= (player->GetRightWall(wallOffset).body!=nullptr )-(player->GetLeftWall(wallOffset).body!=nullptr);
	if(player->GetIsOnFloor()==false && wallSide!=0  ){
		wallMode=true;
		if(player->GetIsFalling() ){
			player->SetGravityMultiplier(0.3);
		}else{
			player->SetGravityMultiplier(1.0);
		}
	}else{
		player->SetGravityMultiplier(1.0);
	}

	
	//Jumps
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)){
		if (wallMode==true){
			//Wall Jump
			if (player->GetIsJumpReleased() ){
				player->Jump(5.0,true);
				player->SetControllerHorizontalVelocity(QVector::Right()*10.0f*-wallSide);
			}
			
		}else{
			//Default Jump
			player->Jump(5.0);
		}
		
	}else{
		player->ReleaseJump();
	}
	

	

	

}


void ExampleScenePlatformer::OnPlatformStep(QBody *body){
	QVector diffVector=mBlockTargetPos-mBlock->GetPosition();
	QVector diffNormal=diffVector.Normalized();

	if(diffVector.Length()<=mBlockMoveSpeed){
		mBlock->AddForce(diffVector);
		mBlockTargetPos=mBlockTargetPos==mBlockStartPos ? mBlockEndPos:mBlockStartPos;

	}
	QVector moveVel=diffNormal*mBlockMoveSpeed;
	mBlock->AddForce(moveVel);



}


void ExampleScenePlatformer::OnCointCollisionEnter(QAreaBody *body,QBody *collidedBody ){
	world->RemoveBody(body);
}


void ExampleScenePlatformer::OnMouseMoved(QVector mousePosition)
{
	QExampleScene::OnMouseMoved(mousePosition);
}
