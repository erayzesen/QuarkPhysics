
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
#include "qaabb.h"
#include "qbody.h"
#include "qmanifold.h"
#include "qcollision.h"




class QBroadPhase {   
public:
    QBroadPhase(){};
    QBroadPhase(float cellSize);

    struct bodyPairHash {
		size_t operator()(const std::pair<QBody*, QBody*>& p) const {
			std::size_t h1 = std::hash<QBody*>{}(p.first);
			std::size_t h2 = std::hash<QBody*>{}(p.second);
			return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
		}
	};

	struct bodyPairEqual {
		bool operator()(const std::pair<QBody*, QBody*>& p1, const std::pair<QBody*, QBody*>& p2) const {
			return (p1.first == p2.first && p1.second == p2.second) ||
				(p1.first == p2.second && p1.second == p2.first);
		}
	};



    
    void insert(QBody *body, const QAABB& aabb);
    void update(QBody *body, const QAABB& newAABB);
    void remove(QBody *body, const QAABB& aabb);

    vector<vector<QBody*> > GetBodiesFromCells();

    void GetAllPairs(unordered_set<pair<QBody*,QBody*>,QBroadPhase::bodyPairHash,bodyPairEqual > &pairs);

    void ApplySweepAndPruneToCells();

    void GetPotentialCollisions(QBody *body,unordered_set<QBody*> &collection);

    




private:
    float cellSize;
    std::unordered_map<int, std::vector<QBody*>> hashTable;
    std::vector<int> getCellKeys(QAABB aabb);
    

    

	 
};


#endif // QBROADPHASE_H
