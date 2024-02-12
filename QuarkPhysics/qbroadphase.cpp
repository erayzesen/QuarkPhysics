
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

#include "qbroadphase.h"
#include <iostream>
#include <algorithm>
#include "qworld.h"
#include "qaabb.h"



QBroadPhase::QBroadPhase(float cellSize) : cellSize(cellSize) {}

void QBroadPhase::Insert(int id, const QAABB& AABB) {
    std::vector<int> cellKeys = GetCellKeys(AABB);

    for (int key : cellKeys) {
        hashTable[key].push_back(id);
    }
    
}

void QBroadPhase::Update(int id, const QAABB& newAABB,QAABB& prevAABB) {
    
    //Debug Test Bounding Boxes

    /* body->GetWorld()->gizmos.push_back( new QGizmoRect(body->spatialContainerAABB) );
    body->GetWorld()->gizmos.push_back( new QGizmoRect(newAABB) ); */

    if(isCleared==false)
        if(prevAABB.isContain(newAABB) )
            return;

    auto fatAABB=newAABB.FattedWithRate(1.2f);
    

    Remove(id, prevAABB);

    
    Insert(id, fatAABB);

    prevAABB=fatAABB;
}

void QBroadPhase::Remove(int id, const QAABB& AABB) {
    std::vector<int> cellKeys = GetCellKeys(AABB);

    for (int key : cellKeys) {
        auto& cell = hashTable[key];
        auto it=find(cell.begin(),cell.end(),id);
        if(it!=cell.end() ){
            cell.erase(it);
        }
    }
}

void QBroadPhase::Clear(vector<QBody *> bodyCollection)
{
    for (auto body: bodyCollection){
        body->spatialContainerAABB=QAABB();
    }
    hashTable.clear();
    isCleared=true;
}



void QBroadPhase::GetAllPairs( unordered_set<pair<int,int>,QBroadPhase::NumericPairHash,QBroadPhase::NumericPairEqual > &pairs,vector<QBody*> &originalCollection){

    
    

    for (auto& cellPair : hashTable) {
        vector<int> &cell = cellPair.second;

        if(cell.size()==1) continue;

         for (auto itA = cell.begin(); itA != cell.end(); ++itA) {
            int idA=*itA;
            QBody * body=originalCollection[idA];
            if(body->GetEnabled()==false )
				continue;
            for (auto itB = itA+1; itB != cell.end(); ++itB) {
                int idB=*itB;
                QBody *otherBody=originalCollection[idB];

                if(otherBody->GetEnabled()==false )
                    continue;

                if( QBody::CanCollide(body,otherBody)==false){
                    continue;
                }

                body->GetWorld()->debugAABBTestCount+=1;
                if(body->GetAABB().isCollidingWith(otherBody->GetAABB()) ){
                    pairs.insert(pair<int,int>{idA,idB});
                }

                
            }
        }
        
    }

    isCleared=false;


}


vector<int> QBroadPhase::GetCellItems(QAABB &aabb){
    vector<int> items;
    vector<int > cellKeys=GetCellKeys(aabb);

    for (int key : cellKeys) {
        vector<int> &cell=hashTable[key];
        items.insert(items.end(),cell.begin(),cell.end() );
    }

    return items;
}

std::vector<int> QBroadPhase::GetCellKeys(QAABB aabb) {
    std::vector<int> cellKeys;

    int minCellX = static_cast<int>(aabb.GetMin().x / cellSize);
    int minCellY = static_cast<int>(aabb.GetMin().y / cellSize);
    int maxCellX = static_cast<int>(aabb.GetMax().x / cellSize);
    int maxCellY = static_cast<int>(aabb.GetMax().y / cellSize);

    // AABB'nin kapsadığı hücreleri buluyoruz.
    for (int cellX = minCellX; cellX <= maxCellX; ++cellX) {
        for (int cellY = minCellY; cellY <= maxCellY; ++cellY) {
            int key = cellX | (cellY << 16);
            cellKeys.push_back(key);
        }
    }

    return cellKeys;
}
