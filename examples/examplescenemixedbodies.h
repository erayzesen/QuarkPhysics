#ifndef EXAMPLESCENEMIXEDBODIES_H
#define EXAMPLESCENEMIXEDBODIES_H

#include "../qexamplescene.h"

class ExampleSceneMixedBodies : public QExampleScene
{
public:
	ExampleSceneMixedBodies(QVector sceneSize);
	void OnMousePressed(QVector mousePosition);
};

#endif // EXAMPLESCENEMIXEDBODIES_H
