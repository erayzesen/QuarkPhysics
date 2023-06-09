
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

#include "examplesceneraycasts.h"
#include "../QuarkPhysics/qraycast.h"
#include "math.h"

ExampleSceneRaycasts::ExampleSceneRaycasts(QVector sceneSize):QExampleScene(sceneSize)
{
	world->SetGravity(QVector(0,0));

	QVector raycastPos=QVector(400,400);
	float raycastLength=1000.0f;
	int raycastCount=90;
	float anglePart=(M_PI*2.0f)/raycastCount;
	for(int i=0;i<raycastCount;i++){
		float angle=anglePart*i;
		QVector vec=raycastLength*QVector(std::cos(angle),std::sin(angle));
		QRaycast *rc=new QRaycast(raycastPos,vec,true);
		world->AddRaycast(rc);
		raycastList.push_back(rc);
	}

	CreateSceneBorders();

	int bodyCount=15;


	for(int i=0;i<bodyCount;i++){
		int bodyType=RandomRange(0,4);
		if(bodyType==0){
			//rect
			QVector rectSize=QVector(RandomRange(32,128),RandomRange(32,128) );
			QRigidBody *rectBody=AddRectBody(RandomRange(rectSize.x,sceneSize.x-rectSize.x),RandomRange(rectSize.y,sceneSize.y-rectSize.y),rectSize.x,rectSize.y );

		}else if(bodyType==1){
			float polygonRadius=RandomRange(16,64);
			int sideCount=RandomRange(6,12);
			QRigidBody *polygonBody=AddPolygonBody(RandomRange(polygonRadius,sceneSize.x-polygonRadius),RandomRange(polygonRadius,sceneSize.y-polygonRadius), polygonRadius,sideCount );

		}else if(bodyType==2){
			float circleRadius=(float)RandomRange(16,64);
			QRigidBody *circleBody=AddCircleBody(RandomRange(circleRadius,sceneSize.x-circleRadius),RandomRange(circleRadius,sceneSize.y-circleRadius), circleRadius);

		}
	}

	


}



void ExampleSceneRaycasts::OnMouseMoved(QVector mousePosition)
{
	QExampleScene::OnMouseMoved(mousePosition);
	for(auto raycast:raycastList){
		raycast->SetPosition(mousePosition);
	}

}

void ExampleSceneRaycasts::OnUpdate()
{
	for(auto raycast:raycastList){
		raycast->SetRotation(raycast->GetRotation()+0.001f);
	}
}
