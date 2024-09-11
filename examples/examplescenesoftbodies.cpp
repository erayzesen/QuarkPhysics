
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

#include "examplescenesoftbodies.h"
#include "cmath"

ExampleSceneSoftBodies::ExampleSceneSoftBodies(QVector sceneSize) :QExampleScene(sceneSize)
{
	world->SetSleepingEnabled(false);
	//world->SetTimeScale(0.1f);
	//Adding Floor
	QRigidBody *floor=new QRigidBody();
	floor->AddMesh(QMesh::CreateWithRect(QVector(3000,64),QVector(0.0f,0.0f) ) )->SetPosition(QVector(512.0f,550.0f));
	floor->SetMode(QBody::Modes::STATIC);
	world->AddBody(floor);
	

	//Adding mass-spring model soft body via only particles
	QSoftBody *pbdBody=new QSoftBody();
	pbdBody->AddMesh( QMesh::CreateWithRect( QVector(128,128),QVector::Zero(),QVector(6,6),true,false,8.0f ) );
	pbdBody->SetRigidity(0.3f)->SetPosition(QVector(150,100))->SetMass(1.0f);
	pbdBody->SetParticleSpesificMassEnabled(true)->SetParticleSpesificMass(0.1f);
	pbdBody->SetShapeMatchingEnabled(true);
	world->AddBody(pbdBody);

	//Adding mass-spring model soft body via gridded rectangle mesh
	QSoftBody *rsBody=new QSoftBody();
	rsBody->AddMesh( QMesh::CreateWithRect( QVector(128,128),QVector::Zero(),QVector(3,3),true ) );
	rsBody->SetRigidity(0.1f)->SetPosition(QVector(500,100));
	rsBody->SetShapeMatchingEnabled(true)->SetShapeMatchingRate(0.1f);
	world->AddBody(rsBody);

	//Adding mass-spring model soft body without polar grids
	QSoftBody *simplePolyBody=new QSoftBody();
	simplePolyBody->AddMesh( QMesh::CreateWithPolygon(64,12,QVector::Zero(),0) );
	simplePolyBody->SetRigidity(0.1f)->SetPosition(QVector(350,0))->SetMass(0.5f);
	world->AddBody(simplePolyBody);

	//Adding mass-spring model soft body via gridded polygon mesh
	QSoftBody *griddedPolyBody=new QSoftBody();
	griddedPolyBody->AddMesh( QMesh::CreateWithPolygon(64,11,QVector::Zero(),2 ) );
	griddedPolyBody->SetRigidity(0.08f)->SetPosition(QVector(700,100))->SetMass(1.0f);
	griddedPolyBody->SetShapeMatchingEnabled(true)->SetShapeMatchingRate(0.2f);
	world->AddBody(griddedPolyBody);

	//Adding pressure volume model soft body via gridded polygon mesh
	QSoftBody *griddedPressuredBody=new QSoftBody();
	griddedPressuredBody->AddMesh( QMesh::CreateWithPolygon(64,12,QVector::Zero(),2 ) );
	griddedPressuredBody->SetRigidity(0.5f)->SetPosition(QVector(900,150))->SetMass(0.5f);
	griddedPressuredBody->SetAreaPreservingEnabled(true)->SetPassivationOfInternalSpringsEnabled(true);
	world->AddBody(griddedPressuredBody);

	float circleRadius=24.0;
	QRigidBody *circleBody=AddCircleBody(500.0f,-200.0f, circleRadius);
	circleBody->SetRestitution(0.9f);


}
