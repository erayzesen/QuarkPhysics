#ifndef QBROADPHASE_H
#define QBROADPHASE_H

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "qbody.h"
#include "qvector.h"

class QBroadPhase
{

	int hashFunction(int x, int y){
		return x+31*y;
	}
	QVector inverseCellSize;

	struct pairHash {
		size_t operator()(const std::pair<QBody*, QBody*> &p) const {
			 return std::hash<QBody*>()(p.first) + std::hash<QBody*>()(p.second);
		}
	};

	struct pairEqual {
		bool operator()(const std::pair<QBody*, QBody*> &p1, const std::pair<QBody*, QBody*> &p2) const {
			return ( (p1.first == p2.first && p1.second == p2.second) ) ;

		}
	};

	void RemoveBodyFromCells(QAABB referenceAABB,QBody *body);

	bool isBodyAdded=false;


public:
	QVector cellSize;

	unordered_map<int,unordered_set<QBody*>> collisionGroups;
	QBroadPhase(QVector cellSize): cellSize(cellSize) {
		inverseCellSize=QVector(1/cellSize.x,1/cellSize.y);
	};
	~QBroadPhase();

	QBroadPhase* Add(QBody *body);

	QBroadPhase* Clear();

	std::unordered_set<std::pair<QBody*,QBody*>,pairHash,pairEqual> pairList;

	std::unordered_set<std::pair<QBody*,QBody*>,pairHash,pairEqual>* GetPairs();



	//Methods




};

#endif // QBROADPHASE_H
