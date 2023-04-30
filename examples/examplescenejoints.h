#ifndef EXAMPLESCENEJOINTS_H
#define EXAMPLESCENEJOINTS_H
#include "../qexamplescene.h"

class ExampleSceneJoints : public QExampleScene
{
public:
	ExampleSceneJoints(QVector sceneSize);
	void PinJointSample(QVector pos);
	void DistanceJointSample(QVector pos);
	void SpringDistanceJointSample(QVector pos);
	void GrooveJointSample(QVector pos);


};

#endif // EXAMPLESCENEJOINTS_H
