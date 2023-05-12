
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

#include "examplescenebenchmarkboxes.h"
#include <SFML/Window/Keyboard.hpp>
#include <functional>

ExampleSceneBenchmarkBoxes::ExampleSceneBenchmarkBoxes(QVector sceneSize):QExampleScene(sceneSize)
{
	//world->SetEnableSleeping(false);
	//BOXES EXAMPLE
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


	int boxGroupCount=20;
	int boxHeapCount=10;
	float boxSize=32.0f;

	auto startPos=floor->GetPosition()+QVector((boxGroupCount*boxSize)*-0.5f,-(32.0f+boxSize*0.5f));
	for(int i=0;i<boxHeapCount;i++){
		float ny=startPos.y-(i*(boxSize));
		for(int n=0;n<boxGroupCount;n++){
			//if(n%2==0)continue;
			float nx=startPos.x+(n*(boxSize));
			AddRectBody(nx,ny,boxSize,boxSize);
		}
	}
}
