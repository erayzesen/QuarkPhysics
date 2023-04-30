#ifndef QEXAMPLESCENE_H
#define QEXAMPLESCENE_H
#include "QuarkPhysics/qworld.h"
#include <iostream>
#include <SFML/Window/Keyboard.hpp>

class QExampleScene
{
protected:

public:
	QExampleScene(QVector sceneSize);
	~QExampleScene(){
		delete world;
	};
	QWorld * world;

	QRigidBody* AddRectBody(float posX,float posY,float width,float height);
	QRigidBody* AddCircleBody(float posX,float posY,float radius);
	QRigidBody* AddPolygonBody(float posX,float posY,float radius,int sideCount);

	QRigidBody* AddRectBody(float posX,float posY);
	QRigidBody* AddCircleBody(float posX,float posY);
	QRigidBody* AddPolygonBody(float posX,float posY,int sideCount);

	QSoftBody* AddBlobBody(float posX,float posY,float radius,int sideCount);


	virtual void OnMousePressed(QVector mousePosition);
	virtual void OnMouseReleased(QVector mousePosition);
	virtual void OnMouseMoved(QVector mousePosition);
	virtual void OnKeyPressed(sf::Keyboard::Key key);
	virtual void OnKeyReleased(sf::Keyboard::Key key);
	virtual void OnUpdate(){};

	float spawnRectSize=32.0f;
	float spawnCircleRadius=24.0f;
	float spawnPolygonRadius=24.0f;

	QVector sceneSize=QVector(1024,600);

	void CreateSceneBorders();

	int RandomRange(int rangeMin,int rangeMax);

	QJoint *mouseJoint=nullptr;
	QSpring *mouseSpring=nullptr;
	QParticle mousePointParticle=QParticle(0,0);

	void CreateJointOrSpringBetweenMouseAndBody(QVector mousePosition);
	void RemoveMouseJointOrSpring();

};

#endif // QEXAMPLESCENE_H
