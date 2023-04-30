#ifndef EXAMPLESCENERAYCASTS_H
#define EXAMPLESCENERAYCASTS_H
#include "../qexamplescene.h"

class ExampleSceneRaycasts: public QExampleScene
{
public:
	ExampleSceneRaycasts(QVector sceneSize);
	vector<QRaycast *> raycastList;
	vector<QBody *> bodyList;

	void OnMouseMoved(QVector mousePosition);
	void OnUpdate();
};

#endif // EXAMPLESCENERAYCASTS_H
