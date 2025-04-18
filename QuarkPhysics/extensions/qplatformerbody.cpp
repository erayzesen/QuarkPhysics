
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

#include "qplatformerbody.h"
#include "../qworld.h"
#include "../qmanifold.h"






QPlatformerBody::QPlatformerBody()
{
    //SetKinematicEnabled(true);
	SetIntegratedVelocitiesEnabled(false);
    SetKinematicCollisionsEnabled(true);
    SetFixedRotationEnabled(true);
	SetFriction(0.0);
	SetStaticFriction(0.5);
}




QPlatformerBody *QPlatformerBody::SetMovingFloorSnapOffset(float value)
{
	movingFloorSnapOffset=value;
    return this;
}

float QPlatformerBody::GetMovingFloorSnapOffset()
{
    return movingFloorSnapOffset;
}

QPlatformerBody *QPlatformerBody::SetGravity(QVector value)
{
    gravity=value;
	upDirection=-gravity.Normalized();
	rightDirection=-upDirection.Perpendicular();
	return this;
}

QVector QPlatformerBody::GetGravity()
{
    return gravity;
}

QPlatformerBody *QPlatformerBody::SetGravityMultiplier(float value)
{
	gravityMultiplier=value;
    return this;
}

float QPlatformerBody::GetGravityMultiplier()
{
    return gravityMultiplier;
}

QPlatformerBody *QPlatformerBody::SetWalkSpeed(float value)
{
	walkSpeed=value;
    return this;
}

float QPlatformerBody::GetWalkSpeed()
{
    return walkSpeed;
}

QPlatformerBody *QPlatformerBody::SetWalkAcelerationRate(float value)
{
	walkAccelerationRate=value;
    return this;
}

float QPlatformerBody::GetWalkAcelerationRate()
{
    return walkAccelerationRate;
}

QPlatformerBody *QPlatformerBody::SetWalkDecelerationRate(float value)
{
	walkDecelerationRate=value;
    return this;
}


QPlatformerBody *QPlatformerBody::SetJumpDurationFrameCount(int value)
{
	jumpDurationFrameCount=value;
    return this;
}

int QPlatformerBody::GetJumpDurationFrameCount()
{
    return jumpDurationFrameCount;
}

QPlatformerBody *QPlatformerBody::SetJumpGravityMultiplier(float value)
{
	jumpGravityMultiplier=value;
    return this;
}

float QPlatformerBody::GetJumpGravityMultiplier()
{
    return jumpGravityMultiplier;
}

QPlatformerBody *QPlatformerBody::SetJumpFallGravityMultiplier(float value)
{
	jumpFallGravityMultiplier=value;
    return this;
}

float QPlatformerBody::GetJumpFallGravityMultiplier()
{
    return jumpFallGravityMultiplier;
}

QPlatformerBody *QPlatformerBody::SetMaxJumpCount(int value)
{
	maxJumpCount=value;
    return this;
}

int QPlatformerBody::GetMaxJumpCount()
{
    return maxJumpCount;
}

QPlatformerBody *QPlatformerBody::SetSpecificPlatformLayers(int layersBit)
{
	platformLayersBit=layersBit;
    return this;
}

int QPlatformerBody::GetSpecificPlatformLayers()
{
    return platformLayersBit;
}

QPlatformerBody *QPlatformerBody::SetFloorMaxAngle(float value)
{
	maxFloorAngle=value;
    return this;
}

float QPlatformerBody::GetFloorMaxAngle()
{
    return maxFloorAngle;
}

QPlatformerBody *QPlatformerBody::SetFloorMaxAngleDegree(float value)
{
	SetFloorMaxAngle(value*(M_PI/180));
    return this;
}

float QPlatformerBody::GetFloorMaxAngleDegree()
{
    return maxFloorAngle/(M_PI/180);
}

QPlatformerBody::CollisionTestInfo QPlatformerBody::GetPlatformCollisions(QVector testPosition,bool FilterByMovingDirection)
{
	CollisionTestInfo res;
    QVector tempPosition=GetPosition();
	SetPosition(testPosition);
	QVector testMovementDirection=(testPosition-tempPosition).Normalized();
	vector<QManifold> manifolds=world->TestCollisionWithWorld(this);
	SetPosition(tempPosition);
	
	
	float minDistance=world->MAX_WORLD_SIZE;
	for (size_t i=0;i<manifolds.size();++i ){
		QManifold manifold=manifolds[i];
		QBody * collidedBody=manifold.bodyA==this ? manifold.bodyB : manifold.bodyA;
		if(collidedBody->GetBodyType()==QBody::BodyTypes::AREA){
			continue;
		}
		if(platformLayersBit!=0){
			if((platformLayersBit & collidedBody->GetLayersBit() )==0){
				continue;
			}
		}
		for(size_t j=0;j<manifold.contacts.size();++j ){
			QCollision::Contact* contact=manifold.contacts[j];
			if( FilterByMovingDirection==true  ){
				QVector collisionNormal=contact->normal;
				if(contact->particle->GetOwnerMesh()->GetOwnerBody()!=this ){
					collisionNormal=-collisionNormal;
				}
				if(collisionNormal.Dot(testMovementDirection)>=0 ){
					continue;
				}
			}
			float distance=contact->position.Dot(testMovementDirection);
			if (distance<minDistance ){
				res.body=collidedBody;
				res.position=contact->position;
				res.normal=contact->particle->GetOwnerMesh()->GetOwnerBody()==this ? contact->normal:-contact->normal;
				res.penetration=contact->penetration;
				minDistance=distance;
				
			}
		}
		
		
	}
		
	
	return res;

}


QPlatformerBody::CollisionTestInfo QPlatformerBody::GetRightWall(float offset)
{
	QVector offsetVector=rightDirection*offset;
	auto collisionTest=GetPlatformCollisions(GetPosition()+offsetVector,true );
	
	if(collisionTest.body!=nullptr ){
		float normalAngle=QVector::AngleBetweenTwoVectors(collisionTest.normal,upDirection);
		if (abs(normalAngle)> maxFloorAngle && abs(normalAngle)<(M_PI-maxFloorAngle) ){
			return collisionTest;
		}
		
	}
    return QPlatformerBody::CollisionTestInfo();

}

QPlatformerBody::CollisionTestInfo QPlatformerBody::GetLeftWall(float offset)
{
    return GetRightWall(-offset);
}

QPlatformerBody::CollisionTestInfo QPlatformerBody::GetFloor(float offset)
{
    
	QVector offsetVector=upDirection*offset;
	auto collisionTest=GetPlatformCollisions(GetPosition()+offsetVector,true );
	
	if(collisionTest.body!=nullptr ){
		float normalAngle=QVector::AngleBetweenTwoVectors(collisionTest.normal,upDirection);
		if ( abs(normalAngle)<=maxFloorAngle ){
			return collisionTest;
		}
		
	}
    return QPlatformerBody::CollisionTestInfo();
}

QPlatformerBody::CollisionTestInfo QPlatformerBody::GetCeiling(float offset)
{
    QVector offsetVector=-upDirection*offset;
	auto collisionTest=GetPlatformCollisions(GetPosition()+offsetVector,true );
	
	if(collisionTest.body!=nullptr ){
		float normalAngle=QVector::AngleBetweenTwoVectors(collisionTest.normal,upDirection);
		if ( abs(normalAngle)> (M_PI-maxFloorAngle)  ){
			return collisionTest;
		}
		
	}
    return QPlatformerBody::CollisionTestInfo();
}

QPlatformerBody *QPlatformerBody::ApplyForce(QVector value)
{
	if(value==QVector::Zero() )
		return this;
	
	//Converting force to horizontal axis
	QVector horizontalForce=value.Dot(rightDirection)*rightDirection;
	//Converting force to vertical axis
	QVector verticalForce=value.Dot(upDirection)*upDirection;

	horizontalVelocity+=horizontalForce;
	verticalVelocity+=verticalForce;

	SetPositionAndCollide(GetPosition()+value);
	WakeUp();

    return this;
}

float QPlatformerBody::GetWalkDecelerationRate()
{
    return walkDecelerationRate;
}

QPlatformerBody *QPlatformerBody::Walk(int side)
{
	walkSide=side;
	
    return this;
}

QPlatformerBody *QPlatformerBody::SetControllerHorizontalVelocity(QVector value)
{
	horizontalVelocity=value;
    return this;
}

QVector QPlatformerBody::GetControllerHorizontalVelocity()
{
    return horizontalVelocity;
}

QPlatformerBody *QPlatformerBody::SetControllerVerticalVelocity(QVector value)
{
	verticalVelocity=value;
    return this;
}

QVector QPlatformerBody::GetControllerVerticalVelocity()
{
    return verticalVelocity;
}

QPlatformerBody *QPlatformerBody::Jump(float force,bool unconditional)
{
	
	if (jumpMode==JumpModes::RELEASED){

		//Jump Conditions
		bool cond1=unconditional;
		bool cond2=onFloor && jumpFrameCountDown>jumpDurationFrameCount;
		bool cond3=onFloor==false && currentJumpCount+1<maxJumpCount;

		if (cond1 || cond2 || cond3){
			jumpMode=JumpModes::PRESSED;
			jumpForce=force;
			jumpFrameCountDown=0;
		}

		if(cond2 || cond3 ){
			currentJumpCount+=1;
		}
		

	}else if (jumpMode==JumpModes::PRESSED){
		jumpMode=JumpModes::PRESSING;
	}
	
	return this;

}

QPlatformerBody *QPlatformerBody::ReleaseJump()
{
	jumpMode=JumpModes::RELEASED;
	
	return this;
}

bool QPlatformerBody::GetIsFalling()
{
    return isFalling;
}

bool QPlatformerBody::GetIsRising()
{
    return isRising;
}



bool QPlatformerBody::GetIsJumpReleased()
{
    return jumpMode==JumpModes::RELEASED;
}

void QPlatformerBody::PostUpdate()
{
	QVector dirCeiling=upDirection;
	QVector dirFloor=-upDirection;
	QVector dirLeftWall=dirCeiling.Rotated(-M_PI/2);
	QVector dirRightWall=dirCeiling.Rotated(M_PI/2);

	

	QVector gravityAmount=gravity*gravityMultiplier;

	if (ignoreGravity){
		gravityAmount=QVector::Zero();
	}

	//Saving Current Position before Tests
	QVector tempPosition=GetPosition();

	//APPLY MOVING PLATFORM FORCES

	bool isOnMovingFloor=false;

	if (lastMovableFloor!=nullptr){
		QVector floorVelocity=lastMovableFloor->GetPosition()-lastMovableFloor->GetPreviousPosition();
		AddPosition(floorVelocity );
		isOnMovingFloor=true;

		//Checking The Last Movable Floor 

		tempPosition=GetPosition();
		AddPosition(dirFloor*movingFloorSnapOffset);
		vector<QCollision::Contact*> contacts= world->GetCollisions(this,lastMovableFloor);
		SetPosition(tempPosition);

		if(contacts.size()==0 ){
			lastMovableFloor=nullptr;
		}else{
			bool is_still_floor=false;
			for(size_t i=0;i<contacts.size();++i ){
				QCollision::Contact * contact=contacts[i];
				QVector normal=contact->particle->GetOwnerMesh()->GetOwnerBody()==this ? contact->normal:-contact->normal;
				float floorAngle=QVector::AngleBetweenTwoVectors(normal,upDirection);
				if( abs(floorAngle)<maxFloorAngle){
					is_still_floor=true;
					break;
				}
			}
			if(is_still_floor==false){
				lastMovableFloor=nullptr;
			}
		}

	}



	//ADDING JUMP VELOCITIES

	if (jumpMode==JumpModes::PRESSED){
		verticalVelocity=jumpForce*upDirection;
	}else if(jumpMode==JumpModes::PRESSING){
		if(verticalVelocity.Dot(upDirection)>0 ){
			gravityAmount*=jumpGravityMultiplier;
		}
	}else if(jumpMode==JumpModes::RELEASED){
		if(verticalVelocity.Dot(upDirection)>0 ){
			gravityAmount*=jumpFallGravityMultiplier;
		}		
	}



	jumpFrameCountDown+=1;

	
	

	//APPLY VERTICAL VELOCITIES
	tempPosition=GetPosition();

	//Checking Floor
	onFloor=false;

	AddPosition(dirFloor*1.0f);
	vector<QManifold>  nextFloorManifolds=world->TestCollisionWithWorld(this);
	SetPosition(tempPosition);


	for(size_t i=0;i<nextFloorManifolds.size();++i){
		QManifold manifold=nextFloorManifolds[i];
		QBody *collidedBody=manifold.bodyA==this ? manifold.bodyB:manifold.bodyA;
		if(collidedBody->GetBodyType()==QBody::BodyTypes::AREA){
			continue;
		}

		if(platformLayersBit!=0){
			if((platformLayersBit & collidedBody->GetLayersBit() )==0){
				continue;
			}
		}

		for(size_t j=0;j<manifold.contacts.size();++j){
			QCollision::Contact * contact=manifold.contacts[j];
			QVector normal=contact->particle->GetOwnerMesh()->GetOwnerBody()==this ? contact->normal:-contact->normal;
			
			float floorAngle=QVector::AngleBetweenTwoVectors(normal,upDirection);
			if( abs(floorAngle)<maxFloorAngle ){
				onFloor=true;
				if(collidedBody->GetBodyType()==QBody::BodyTypes::RIGID){
					QRigidBody *collidedRigidBody=static_cast<QRigidBody*>(collidedBody);
					if(collidedRigidBody!=nullptr){
						lastMovableFloor=collidedRigidBody;
					}
				}
				break;
			}
		}
		if (onFloor==true)
			break;
	}
	

	//Checking Ceiling
	onCeiling=false;

	tempPosition=GetPosition();
	AddPosition(dirCeiling*1.0);
	vector<QManifold>  nextCeilingManifolds=world->TestCollisionWithWorld(this);
	SetPosition(tempPosition);

	
	for(size_t i=0;i<nextCeilingManifolds.size();++i){
		QManifold manifold=nextCeilingManifolds[i];
		QBody *collidedBody=manifold.bodyA==this ? manifold.bodyB:manifold.bodyA;
		if(collidedBody->GetBodyType()==QBody::BodyTypes::AREA){
			continue;
		}
		if(platformLayersBit!=0){
			if((platformLayersBit & collidedBody->GetLayersBit() )==0){
				continue;
			}
		}
		for(size_t j=0;j<manifold.contacts.size();++j){
			QCollision::Contact * contact=manifold.contacts[j];
			QVector normal=contact->particle->GetOwnerMesh()->GetOwnerBody()==this ? contact->normal:-contact->normal;
			
			float floorAngle=QVector::AngleBetweenTwoVectors(normal,upDirection);
			if( abs(floorAngle)> (M_PI-maxFloorAngle) ){
				onCeiling=true;
				break;
			}
		}
		if (onCeiling==true)
			break;
	}
	if(onFloor){
		isRising=false;
		isFalling=false;
		currentJumpCount=0;
	}else if(verticalVelocity.Dot(upDirection)<0){
		isFalling=true;
		isRising=false;
	}else if(verticalVelocity.Dot(upDirection)>0){
		isFalling=false;
		isRising=true;
	}

	if (velocityLimit>0.0f && verticalVelocity.Length()>velocityLimit){
		verticalVelocity=velocityLimit*verticalVelocity.Normalized();
	}

	
	if ( (onFloor==true ) && verticalVelocity.Dot(upDirection)<0 ){
		verticalVelocity=QVector::Zero();
	}else if(onCeiling==true && verticalVelocity.Dot(upDirection)>0 ){
		verticalVelocity=QVector::Zero();
	}else{
		AddPosition(verticalVelocity);
		verticalVelocity+=gravityAmount;
	}
	


	
	// APPLY HORIZONTAL VELOCITIES
	
	if(walkSide==0){
		if(abs(horizontalVelocity.x)<0.001f && abs(horizontalVelocity.y)<0.001f ){
			horizontalVelocity=QVector::Zero();
		}else{
			horizontalVelocity+=-horizontalVelocity*walkDecelerationRate;
		}
	}else{
		horizontalVelocity+=((walkSpeed*walkSide*rightDirection)-horizontalVelocity)*walkAccelerationRate;
	}

	
	if (velocityLimit>0.0f && horizontalVelocity.Length()>velocityLimit){
		horizontalVelocity=velocityLimit*horizontalVelocity.Normalized();
	}

	if (horizontalVelocity!=QVector::Zero() ){
		//Requested Target Position
		
		QVector walkVector=horizontalVelocity;

		auto GetPlatformCollisionsSlopedFloor=GetPlatformCollisions(GetPosition()+dirFloor*5.0f,true);
		
		if (GetPlatformCollisionsSlopedFloor.body!=nullptr){
			float floorAngle=QVector::AngleBetweenTwoVectors(GetPlatformCollisionsSlopedFloor.normal,upDirection);
			if (abs(floorAngle)<=maxFloorAngle ){
				QVector floorUnit=-GetPlatformCollisionsSlopedFloor.normal.Perpendicular();
				walkVector=walkVector.Dot(floorUnit )*floorUnit;
				//cout<<"there is a slope ground: new velocity is :"<<floorUnit<<endl;
			}
			
		}
		
		
		AddPosition(walkVector);
		
	}

	//Horizontal Move and Wall Collision Tests
	CollisionTestInfo rightWallTest=GetRightWall(0.0f);
	if(rightWallTest.body!=nullptr ){
		if (horizontalVelocity.Dot(rightDirection)>0 && rightWallTest.body->GetMode()==QBody::STATIC ){
			horizontalVelocity=QVector::Zero();
			QVector defaultResponse=(rightWallTest.normal*rightWallTest.penetration);
			QVector onAxisResponse=defaultResponse.Dot(rightDirection)*rightDirection;
			SetPosition( GetPosition()+onAxisResponse,true );
		}
	}

	CollisionTestInfo leftWallTest=GetLeftWall(0.0f);
	if(leftWallTest.body!=nullptr ){
		if (horizontalVelocity.Dot(rightDirection)<0 && leftWallTest.body->GetMode()==QBody::STATIC){
			horizontalVelocity=QVector::Zero();
			QVector defaultResponse=(leftWallTest.normal*leftWallTest.penetration);
			QVector onAxisResponse=defaultResponse.Dot(-rightDirection)*-rightDirection;
			SetPosition( GetPosition()+onAxisResponse,true );
		}
	}

	

}

bool QPlatformerBody::GetIsOnFloor()
{
    return onFloor;
}

bool QPlatformerBody::GetIsOnCeiling()
{
    return onCeiling;
}



