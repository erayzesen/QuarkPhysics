#include "qareabody.h"
#include "qbody.h"
#include "qcollision.h"
#include "qworld.h"

QAreaBody::QAreaBody(){
	simulationModel=QBody::SimulationModels::RIGID_BODY;
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
