#include "examplescenebenchmarkboxes2.h"

ExampleSceneBenchmarkBoxes2::ExampleSceneBenchmarkBoxes2(QVector sceneSize):QExampleScene(sceneSize)
{
	world->SetGravity(QVector(0.0f,0.1f));
	spawnRectSize=16.0f;
	spawnCircleRadius=8.0f;
	world->SetEnableSleeping(true);
	//BOXES EXAMPLE
	QBody *floor=new QBody();
	floor->AddMesh(QMesh::CreateWithRect(QVector(5000,64),QVector(0.0f,0.0f) ) )->SetPosition(QVector(512.0f,550.0f));
	floor->SetMode(QBody::Modes::STATIC);
	world->AddBody(floor);


	int boxGroupCount=30;
	int boxHeapCount=20;
	float boxSize=16.0f;

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
