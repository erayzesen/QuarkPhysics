
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
#include "qareabody.h"
#include "qbody.h"
#include "qcollision.h"
#include "qworld.h"

QAreaBody::QAreaBody(){
	simulationModel=QBody::SimulationModels::RIGID_BODY;
	bodyType=QBody::BodyTypes::AREA;
}
void QAreaBody::AddCollidedBody(QBody* body){
	bool isBodyNew=bodies.insert(body).second;
	if(isBodyNew){
		OnCollisionEnter(body);
		if(CollisionEnterEventListener!=nullptr)
			CollisionEnterEventListener(this,body);
	}

}
void QAreaBody::CheckBodies(){
	vector<QBody*> blackList;
	for(auto body:bodies){
		if(world->GetCollisions(this,body).size()==0 ){
			blackList.push_back(body);
		}
	}
	while (blackList.size()!=0) {
		OnCollisionExit(blackList[0]);
		if(CollisionExitEventListener!=nullptr)
			CollisionExitEventListener(this,blackList[0]);
		bodies.erase(blackList[0]);
		blackList.erase(blackList.begin());
	}
}
