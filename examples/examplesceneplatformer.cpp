
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
	//Adding Floors
	QRigidBody *floor;
	floor=new QRigidBody();
	floor->AddMesh(QMesh::CreateWithRect(QVector(400,64) ) )->SetPosition(QVector(512.0f,550.0f));
	floor->SetMode(QBody::Modes::STATIC);
	world->AddBody(floor);

	QVector playerStartPosition=floor->GetPosition()+QVector(0,-48);

	floor=new QRigidBody();
	floor->AddMesh(QMesh::CreateWithRect(QVector(128,64) ) )->SetPosition(QVector(100.0f,400.0f));
	floor->SetMode(QBody::Modes::STATIC);
	world->AddBody(floor);

	//Adding Walls
	QRigidBody *wall;
	wall=new QRigidBody();
	wall->AddMesh(QMesh::CreateWithRect(QVector(64,300) ) )->SetPosition(QVector(780.0f,320.0f));
	wall->SetMode(QBody::Modes::STATIC);
	world->AddBody(wall);

	wall=new QRigidBody();
	wall->AddMesh(QMesh::CreateWithRect(QVector(128,250) ) )->SetPosition(QVector(600.0f,295.0f));
	wall->SetMode(QBody::Modes::STATIC);
	world->AddBody(wall);

	//Adding a Moveable Platform
	mBlock=new QRigidBody();
	mBlock->AddMesh(QMesh::CreateWithRect( QVector(128,32)) )->SetPosition(QVector(430.0f,180.0f));
	mBlock->SetKinematicEnabled(true)->SetFixedRotationEnabled(true);
	mBlockStartPos=mBlock->GetPosition();
	mBlockEndPos=mBlockStartPos+QVector(-130,200);
	mBlockTargetPos=mBlockEndPos;
	world->AddBody(mBlock);
	mBlock->PreStepEventListener=std::bind(&ExampleScenePlatformer::OnPlatformPreStep,this,std::placeholders::_1);



	//Adding Player
	player=new QRigidBody();
	player->AddMesh( QMesh::CreateWithRect(QVector(32,32) ) )->SetPosition(playerStartPosition );
	player->SetKinematicEnabled(true)->SetFixedRotationEnabled(true)->SetKinematicCollisionsEnabled(true);
	world->AddBody(player);
	player->PreStepEventListener=std::bind(&ExampleScenePlatformer::OnPlayerPreStep,this,std::placeholders::_1);
	player->StepEventListener=std::bind(&ExampleScenePlatformer::OnPlayerStep,this,std::placeholders::_1);
	player->CollisionEventListener=std::bind(&ExampleScenePlatformer::OnPlayerCollision,this,placeholders::_1,placeholders::_2);

	//Adding Coints(Using QAreaBody)
	QAreaBody *coint;
	coint=new QAreaBody();
	coint->AddMesh( QMesh::CreateWithCircle(8.0f)  )->SetPosition(QVector(350,430) );
	coint->CollisionEnterEventListener=bind(&ExampleScenePlatformer::OnCointCollisionEnter,this,placeholders::_1,placeholders::_2);
	world->AddBody(coint);

	coint=new QAreaBody();
	coint->AddMesh( QMesh::CreateWithCircle(8.0f)  )->SetPosition(QVector(386,430) );
	coint->CollisionEnterEventListener=bind(&ExampleScenePlatformer::OnCointCollisionEnter,this,placeholders::_1,placeholders::_2);
	world->AddBody(coint);
	
	coint=new QAreaBody();
	coint->AddMesh( QMesh::CreateWithCircle(8.0f)  )->SetPosition(QVector(420,430) );
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



void ExampleScenePlatformer::OnPlayerPreStep(QBody *body)
{
	
	//Player Movement Properties
	float moveMaxSpeed=5.0f;
	float moveAccelerationRate=0.1f;
	float moveDecelarationRate=0.3f;
	float jumpForce=5.0f;
	float wallJumpHorizontalForce=16.0f;
	float wallFrictionFactor=0.2f;

	float gravity=0.2f;



	//Defining moveAxis according to keyboard input
	int moveAxis=(sf::Keyboard::isKeyPressed(sf::Keyboard::Right) )-(sf::Keyboard::isKeyPressed(sf::Keyboard::Left) );

	//Adding gravity if player doesn't on the platform
	if(isOnFloor==false)
		if( ( (isOnRightWall && moveAxis==1) || (isOnLeftWall && moveAxis==-1) ) && velocity.y>0.0f ) 
			//When player is sticking the wall
			velocity.y+=gravity*wallFrictionFactor;
		else
			//When the player on the air
			velocity.y+=gravity;
	else
		//When the player on the floor
		velocity.y=0;

	if(isOnCeiling==true)
		//If is the player collided with the ceiling
		velocity.y=0;

	//Horizontal movements of player
	if(moveAxis==0){ // If doesn't exist the input
		velocity.x+=-velocity.x*moveDecelarationRate;
	}else{
		//Adding horizontal velocity
		velocity.x+=((moveMaxSpeed*moveAxis)-velocity.x)*moveAccelerationRate;
	}


	//Jump and Wall Jump
	if(jumpTickDown==0){
		bool isJumpPressed=sf::Keyboard::isKeyPressed(sf::Keyboard::Up);
		if(isJumpPressed){
			if(isOnFloor){
				velocity.y-=jumpForce;
				jumpTickDown=jumpTickCount;
			}
			if(isOnRightWall){
				velocity.y-=jumpForce;
				velocity.x-=wallJumpHorizontalForce;
				jumpTickDown=jumpTickCount;
			}
			if(isOnLeftWall){
				velocity.y-=jumpForce;
				velocity.x+=wallJumpHorizontalForce;
				jumpTickDown=jumpTickCount;
			}
		}

	}


	jumpTickDown=max(0,jumpTickDown-1);

	player->AddPosition(velocity);






	//Reset side checkers
	isOnFloor=false;
	isOnCeiling=false;
	isOnLeftWall=false;
	isOnRightWall=false;


}
void ExampleScenePlatformer::OnPlayerStep(QBody *body){
	if(currentFloor!=nullptr){
		//Checking if  the fatted player's aaabb is still colliding with currentfloor's aabb
		if(player->GetAABB().Fattened(1.0f).isCollidingWith(currentFloor->GetAABB())==false ){
			currentFloor=nullptr;
		}else{
			//Adding the floor platform's force to player
			player->SetForce(currentFloor->GetForce());
		}
	}
}


bool ExampleScenePlatformer::OnPlayerCollision(QBody *body, QBody::CollisionInfo info)
{
	//Floor properties
	auto side=QVector::GetVectorSide(-info.normal,QVector::Up());

	if(info.body->GetBodyType()!=QBody::BodyTypes::RIGID){
		return false;
	}
	//cout<<side<<endl;
	if(side==QSides::DOWN){
		isOnFloor=true;
		//Defining currentfloor 
		currentFloor=static_cast<QRigidBody*>(info.body);
	}else if(side==QSides::RIGHT){
		isOnRightWall=true;
	}else if(side==QSides::LEFT){
		isOnLeftWall=true;
	}else if(side==QSides::UP){
		isOnCeiling=true;
	}

//	string sideArr[5]={"UP","RIGHT","DOWN","LEFT","NONE"};
//	cout<<" - side:"<<sideArr[side]<<endl;
	return true;

}


void ExampleScenePlatformer::OnPlatformPreStep(QBody *body){
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

//	QVector testVector=mousePosition-player->GetPosition();

//	float rad=QVector::AngleBetweenTwoVectors(testVector,QVector::Up());
//	auto side=QVector::GetVectorSide(testVector,QVector::Left());
//	string sideArr[5]={"UP","RIGHT","DOWN","LEFT","NONE"};
//	cout<<"rad:"<<rad/(M_PI/180)<<" - side:"<<sideArr[side]<<endl;
}
