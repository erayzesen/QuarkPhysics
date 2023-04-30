#include "qbroadphase.h"
#include <cmath>
#include <iostream>
#include <algorithm>



QBroadPhase::~QBroadPhase()
{

}

QBroadPhase *QBroadPhase::Add(QBody* body)
{
	QAABB aabb=body->GetAABB();
	if(body->GetFattedAABB().isContain(aabb)){
		return this;
	}

	RemoveBodyFromCells(body->GetFattedAABB(),body);


	float fatFactor=5.0f;
	if(body->GetSimulationModel()==QBody::RIGID_BODY){
		fatFactor+=(body->GetPosition()-body->GetPreviousPosition()).Length();
	}
	body->GetFattedAABB()=body->GetAABB().Fatted(fatFactor );
	QVector min = body->GetFattedAABB().GetMin();
	min *= inverseCellSize;
	int minX = std::floor(min.x);
	int minY= std::floor(min.y);


	QVector max = body->GetFattedAABB().GetMax();
	max *= inverseCellSize;
	int maxX = std::floor(max.x);
	int maxY= std::floor(max.y);


	for (int x = minX; x <= maxX; x++){
		for (int y = minY; y <= maxY; y++)
		{
			int cell=hashFunction(x,y);

			collisionGroups[cell].insert(body);
		}
	}

	isBodyAdded=true;
	return this;
}





QBroadPhase *QBroadPhase::Clear()
{
	collisionGroups.clear();
	return this;
}

std::unordered_set<std::pair<QBody*,QBody*>,QBroadPhase::pairHash,QBroadPhase::pairEqual>* QBroadPhase::GetPairs()
{
//	if(isBodyAdded==false)
//		return &pairList;

	pairList.clear();

	for(auto const &[key, value] : collisionGroups) {
		for(auto ita=value.begin();ita!=value.end();++ita) {
			auto bodyA=*ita;
			for(auto itb=next(ita);itb!=value.end();++itb) {
				auto bodyB=*itb;
				auto pair = minmax(bodyA, bodyB);
				pairList.insert(pair);

			}
		}
	}
	isBodyAdded=false;
	return &pairList;
}

void QBroadPhase::RemoveBodyFromCells(QAABB referenceAABB, QBody *body)
{
	QVector min = referenceAABB.GetMin();
	min *= inverseCellSize;
	int minX = std::floor(min.x);
	int minY= std::floor(min.y);


	QVector max = referenceAABB.GetMax();
	max *= inverseCellSize;
	int maxX = std::floor(max.x);
	int maxY= std::floor(max.y);

	for (int x = minX; x <= maxX; x++){
		for (int y = minY; y <= maxY; y++)
		{
			int cell=hashFunction(x,y);
			auto groupIt=collisionGroups.find(cell);
			if(groupIt==collisionGroups.end())
				continue;
			unordered_set<QBody*> *group=&groupIt->second;
			//auto it=std::find(group->begin(),group->end(),body);
			auto it=group->find(body);
			if(it!=group->end()){
				group->erase(it);
			}

		}
	}


}

