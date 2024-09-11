
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

#include <iostream>
#include <algorithm>
#include "qspatialhashing.h"
#include "algorithm"



QSpatialHashing::QSpatialHashing(vector<QBody *> &worldBodies, float sizeOfCells) : QBroadPhase(worldBodies)
{
    
}

void QSpatialHashing::Clear()
{
    bodyOldCells.clear();
    cells.clear();
    pairs.clear();
}


void QSpatialHashing::Insert(QBody *body)
{

    auto aabb = body->GetAABB();


    CellAABB cellAABB(floor(aabb.GetMin().x * cellSizeFactor),floor(aabb.GetMin().y * cellSizeFactor),
                        floor(aabb.GetMax().x * cellSizeFactor),floor(aabb.GetMax().y * cellSizeFactor));

    auto it = bodyOldCells.find(body);

    if (it != bodyOldCells.end()) {
            
        CellAABB &oldCellAABB = it->second;

        //AABB Cells doesn't change against to the previous.  
        if (oldCellAABB==cellAABB) {
            
            return;
        }

        //Removing body from cells
        RemoveBodyFromCells(body,oldCellAABB);
        
    }

    bodyOldCells[body]= cellAABB;
    //Adding body to cells
    for (int cellX = cellAABB.minX; cellX <= cellAABB.maxX; ++cellX) {
        for (int cellY = cellAABB.minY; cellY <= cellAABB.maxY; ++cellY) {
            cells[{cellX, cellY}].push_back(body);
        }
    }


}

void QSpatialHashing::Remove(QBody *body)
{

    auto it = bodyOldCells.find(body);

    if (it != bodyOldCells.end()) {
        
        CellAABB &oldCellAABB = it->second;

        //Removing body from cells
        RemoveBodyFromCells(body,oldCellAABB);
        //Removing from bodyOldCells collection 
        bodyOldCells.erase(it);
    }

}

void QSpatialHashing::SetCellSize(float size)
{
    Clear();
    cellSize=size;
    cellSizeFactor=1/cellSize;
}

void QSpatialHashing::RemoveBodyFromCells(QBody *body, CellAABB &cellAABB)
{
    for (int cellX = cellAABB.minX; cellX <= cellAABB.maxX; ++cellX) {
        for (int cellY = cellAABB.minY; cellY <= cellAABB.maxY; ++cellY) {
            auto& cell = cells[{cellX, cellY}];
            auto bodyIt=find(cell.begin(),cell.end(),body );
            if (bodyIt!=cell.end ()){
                cell.erase(bodyIt);
            }
        }
    }
}

std::unordered_set<std::pair<QBody*, QBody*>,QBody::BodyPairHash,QBody::BodyPairEqual> & QSpatialHashing::GetPairs() {
     

    pairs.clear();

    for (auto& cell : cells) {
        std::vector<QBody*>& cellBodies = cell.second;
        size_t numBodies = cellBodies.size();
        if (numBodies<=1){
            continue;
        }

        //Sorting cell bodies to apply Sweep and Prune method.
        sort(cellBodies.begin(),cellBodies.end(),SortBodiesHorizontal);
        
        for (size_t i = 0; i < numBodies - 1; ++i) {
            QBody* bodyA = cellBodies[i];
            
            for (size_t j = i + 1; j < numBodies; ++j) {
                QBody* bodyB = cellBodies[j];
                
                pair<QBody*,QBody*> bodyPair={bodyA,bodyB};
                if (pairs.find(bodyPair)!=pairs.end() ){
                    continue;
                }
                
                if (!BodiesCanCollide(bodyA, bodyB))
                    continue;

                
                if(bodyA->GetAABB().GetMax().x >= bodyB->GetAABB().GetMin().x){
                    if( bodyA->GetAABB().GetMin().y <= bodyB->GetAABB().GetMax().y &&
                        bodyA->GetAABB().GetMax().y >= bodyB->GetAABB().GetMin().y) {

                        pairs.insert(bodyPair);

                    }

                }else{
                    break;
                }
                
            }
        }
    }

    return pairs;

    
}
