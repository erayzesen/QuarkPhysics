
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

void QBroadPhase::insert(QBody *body, const QAABB& aabb) {
    std::vector<int> cellKeys = getCellKeys(aabb);

    for (int key : cellKeys) {
        hashTable[key].push_back(body);
    }
    
}

void QBroadPhase::update(QBody *body, const QAABB& newAABB) {
    
    //Debug Test Bounding Boxes

    /* body->GetWorld()->gizmos.push_back( new QGizmoRect(body->spatialContainerAABB) );
    body->GetWorld()->gizmos.push_back( new QGizmoRect(newAABB) ); */

    if(body->spatialContainerAABB.isContain(newAABB) ){
        return;
    }

    auto fatAABB=newAABB.FattedWithRate(1.2f);
    

    remove(body, body->spatialContainerAABB);

    
    insert(body, fatAABB);

    body->spatialContainerAABB=fatAABB;
}

void QBroadPhase::remove(QBody *body, const QAABB& aabb) {
    std::vector<int> cellKeys = getCellKeys(aabb);

    for (int key : cellKeys) {
        auto& cell = hashTable[key];
        auto it=find(cell.begin(),cell.end(),body);
        if(it!=cell.end() ){
            cell.erase(it);
        }
    }
}

void QBroadPhase::ApplySweepAndPruneToCells()
{
    for (auto& cellPair : hashTable) {
        vector<QBody*> &cell = cellPair.second;
        sort(cell.begin(),cell.end(),QWorld::SortBodies);
    }
}

void QBroadPhase::GetPotentialCollisions(QBody *body, unordered_set<QBody *> &collection)
{
    
    
    if(body->GetEnabled()==false )
        return;

    std::vector<int> cellKeys = getCellKeys(body->spatialContainerAABB);

    for (size_t i = 0; i < cellKeys.size(); ++i){
        auto &cell = hashTable[cellKeys[i]];

        auto itA = cell.begin();
        while (itA != cell.end() && *itA != body) {
            ++itA;
        }
        
        for (auto itB = std::next(itA); itB != cell.end(); ++itB) {
            QBody *otherBody=*itB;

            if(otherBody->GetEnabled()==false )
                continue;

            if( QBody::CanCollide(body,otherBody)==false){
                continue;
            }
            body->GetWorld()->debugAABBTestCount+=1;
            if(body->GetAABB().GetMax().x >= otherBody->GetAABB().GetMin().x){
                if( body->GetAABB().GetMin().y <= otherBody->GetAABB().GetMax().y &&
                    body->GetAABB().GetMax().y >= otherBody->GetAABB().GetMin().y) {
                    collection.insert(otherBody);
                }

            }else{
                break;
            }

            
        }
         

    }
    
}

vector< vector<QBody*> > QBroadPhase::GetBodiesFromCells() {
    vector< vector<QBody*> > result;

    // Hash tablosundaki her hücreyi dolaşıyoruz.
    for (const auto& cellPair : hashTable) {
        const vector<QBody*>& cell = cellPair.second;
        std::vector<QBody*> uniqueBodies(cell.begin(), cell.end());
        result.push_back(uniqueBodies);
    }

    return result;
}

void QBroadPhase::GetAllPairs( unordered_set<pair<QBody*,QBody*>,QBroadPhase::bodyPairHash,bodyPairEqual > &pairs){

    
    

    for (auto& cellPair : hashTable) {
        vector<QBody*> &cell = cellPair.second;

         for (auto itA = cell.begin(); itA != cell.end(); ++itA) {
            QBody *body=*itA;
            if(body->GetEnabled()==false )
				continue;
            for (auto itB = itA+1; itB != cell.end(); ++itB) {
                QBody *otherBody=*itB;

                if(otherBody->GetEnabled()==false )
                    continue;

                /* if( QBody::CanCollide(body,otherBody)==false){
                    continue;
                } */

                body->GetWorld()->debugAABBTestCount+=1;
                if(body->GetAABB().isCollidingWith(otherBody->GetAABB()) ){
                    pairs.insert(pair<QBody*,QBody*>{body,otherBody});
                }

                
            }
        }

        

        
    }


} 

std::vector<int> QBroadPhase::getCellKeys(QAABB aabb) {
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

