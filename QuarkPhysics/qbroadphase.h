
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
