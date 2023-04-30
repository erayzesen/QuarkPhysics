#include "examplescenebenchmarkboxes.h"

ExampleSceneBenchmarkBoxes::ExampleSceneBenchmarkBoxes(QVector sceneSize):QExampleScene(sceneSize)
{
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
