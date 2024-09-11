
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
		if (world!=nullptr){
			delete world;
			world=nullptr;
		}
		
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
