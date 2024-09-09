
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

#include "qexamplescene.h"
#include "QuarkPhysics/qmesh.h"
#include "QuarkPhysics/extensions/qspatialhashing.h"



QExampleScene::QExampleScene(QVector sceneSize)
{
	world=new QWorld();
	//Trying QSpatialHashing broadphase extension
	/* QSpatialHashing *broadPhase=new QSpatialHashing(world->bodies,128.0f);
	world->broadPhase=broadPhase; */
	this->sceneSize=sceneSize;


}



QRigidBody* QExampleScene::AddRectBody(float posX, float posY, float width, float height)
{
	QRigidBody *body=new QRigidBody();
	body->AddMesh(QMesh::CreateWithRect(QVector(width,height),QVector(0.0f,0.0f) ) )->SetPosition(QVector(posX,posY));
	//body->AddMesh(QMesh::CreateRect(QVector(width,height),QVector(-32.0f,0.0f) ) )->SetPosition(posX,posY);
	//body->AddMesh(QMesh::CreateCircle(spawnCircleRadius,QVector(32.0f,0.0f) ) )->SetPosition(posX,posY);
	//float randRot=std::rand()/(RAND_MAX/M_PI);
	world->AddBody(body);
	return body;
}

QRigidBody* QExampleScene::AddCircleBody(float posX, float posY, float radius)
{
	QRigidBody *body=new QRigidBody();
	body->AddMesh(QMesh::CreateWithCircle(radius,QVector::Zero() ) )->SetPosition(QVector(posX,posY));
	//body->AddMesh(QMesh::CreateConvexPolygon(spawnPolygonRadius,6,QVector::Zero()) )->SetPosition(posX,posY);
	world->AddBody(body);
	return body;
}

QRigidBody *QExampleScene::AddPolygonBody(float posX, float posY, float radius, int sideCount)
{
	QRigidBody *body=new QRigidBody();
	body->AddMesh(QMesh::CreateWithPolygon(radius,sideCount,QVector::Zero()) )->SetPosition(QVector(posX,posY));
	world->AddBody(body);
	return body;
}

QRigidBody* QExampleScene::AddRectBody(float posX, float posY)
{
	return AddRectBody(posX,posY,spawnRectSize,spawnRectSize);
}

QRigidBody* QExampleScene::AddCircleBody(float posX, float posY)
{
	return AddCircleBody(posX,posY,spawnCircleRadius);
}

QRigidBody *QExampleScene::AddPolygonBody(float posX, float posY, int sideCount)
{
	return AddPolygonBody(posX,posY,spawnCircleRadius,sideCount);
}

QSoftBody *QExampleScene::AddBlobBody(float posX, float posY, float radius, int sideCount)
{
	QSoftBody *blob=new QSoftBody();
	blob->AddMesh(QMesh::CreateWithPolygon(radius,sideCount,QVector(0,0)))->SetPosition(QVector(posX,posY));
	blob->SetAreaPreservingEnabled(true);
	blob->SetAreaPreservingRate(0.8f);
	//blob->SetRigidity(0.5f);
	world->AddBody(blob);
	return blob;
}

void QExampleScene::CreateSceneBorders()
{
	QVector hBoxSizes=QVector(128,sceneSize.y);
	QVector vBoxSizes=QVector(sceneSize.x,128);

	QRigidBody *bodyLeft=new QRigidBody();
	bodyLeft->AddMesh( QMesh::CreateWithRect(hBoxSizes) )->SetPosition(QVector(-hBoxSizes.x*0.5f,hBoxSizes.y*0.5f));
	bodyLeft->SetMode(QBody::STATIC);
	world->AddBody(bodyLeft);

	QRigidBody *bodyTop=new QRigidBody();
	bodyTop->AddMesh( QMesh::CreateWithRect(vBoxSizes) )->SetPosition(QVector(vBoxSizes.x*0.5f,-vBoxSizes.y*0.5f));
	bodyTop->SetMode(QBody::STATIC);
	world->AddBody(bodyTop);

	QRigidBody *bodyRight=new QRigidBody();
	bodyRight->AddMesh( QMesh::CreateWithRect(hBoxSizes) )->SetPosition(QVector(sceneSize.x+hBoxSizes.x*0.5f,hBoxSizes.y*0.5f));
	bodyRight->SetMode(QBody::STATIC);
	world->AddBody(bodyRight);

	QRigidBody *bodyBottom=new QRigidBody();
	bodyBottom->AddMesh( QMesh::CreateWithRect(vBoxSizes) )->SetPosition(QVector(vBoxSizes.x*0.5f,sceneSize.y+vBoxSizes.y*0.5f));
	bodyBottom->SetMode(QBody::STATIC);
	world->AddBody(bodyBottom);
}

int QExampleScene::RandomRange(int rangeMin, int rangeMax)
{
	int res=rangeMin+rand()%rangeMax;
	return res;
}


//###This side is also an example about controlling objects with mouse
void QExampleScene::CreateJointOrSpringBetweenMouseAndBody(QVector mousePosition){
	if(mouseJoint!=nullptr){
		world->RemoveJoint(mouseJoint);
		delete mouseJoint;
		mouseJoint=nullptr;
	}
	if(mouseSpring!=nullptr){
		world->RemoveSpring(mouseSpring);
		delete mouseSpring;
		mouseSpring=nullptr;
	}
	QRigidBody *targetBody=nullptr;
	QParticle *targetParticle=nullptr;
	vector<QBody*> hitBodies=world->GetBodiesHitByPoint(mousePosition,1,true);
	if(hitBodies.size()>0){
		if(hitBodies[0]->GetMode()!=QBody::STATIC){
			targetBody=static_cast<QRigidBody*>(hitBodies[0]);
		}
	}


	if(targetBody!=nullptr){
		mouseJoint=new QJoint(targetBody,mousePosition,nullptr);
		world->AddJoint(mouseJoint);
	}else{
		vector<QParticle*> hitParticles=world->GetParticlesCloseToPoint(mousePosition,8.0f,1,true);
		if(hitParticles.size()>0){
			targetParticle=hitParticles[0];
		}
	}

	if(targetParticle!=nullptr){
		mouseSpring=new QSpring(targetParticle,&mousePointParticle,0.0f);
		mouseSpring->SetRigidity(1.0f);
		world->AddSpring(mouseSpring);
	}

}


void QExampleScene::OnMousePressed(QVector mousePosition){
	CreateJointOrSpringBetweenMouseAndBody(mousePosition);
}
void QExampleScene::OnMouseReleased(QVector mousePosition){
	if(mouseJoint!=nullptr){
		world->RemoveJoint(mouseJoint);
		delete mouseJoint;
		mouseJoint=nullptr;
	}
	if(mouseSpring!=nullptr){
		world->RemoveSpring(mouseSpring);
		delete mouseSpring;
		mouseSpring=nullptr;
	}
}

void QExampleScene::OnMouseMoved(QVector mousePosition){
	if(mouseJoint!=nullptr){
		mouseJoint->SetAnchorBPosition(mousePosition);
	}
	mousePointParticle.SetGlobalPosition(mousePosition);
}

void QExampleScene::OnKeyPressed(sf::Keyboard::Key key)
{

}

void QExampleScene::OnKeyReleased(sf::Keyboard::Key key)
{

}


