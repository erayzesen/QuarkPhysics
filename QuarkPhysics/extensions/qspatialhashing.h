
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




#ifndef QSPATIALHASHING_H
#define QSPATIALHASHING_H

#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <vector>
#include "../qbroadphase.h"




class QSpatialHashing : public QBroadPhase {
private:
    float cellSize=128.0f;

    struct PairHash {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            return std::hash<T1>()(p.first) ^ 0x100000001b3 ^std::hash<T2>()(p.second);
        };
    };

    struct CellAABB{
        CellAABB(int minimumX,int minimumY,int maximumX,int maximumY){
            minX=minimumX;
            minY=minimumY;
            maxX=maximumX;
            maxY=maximumY;
        };
        CellAABB(){
            minX=0;
            minY=0;
            maxX=0;
            maxY=0;
        }
        int minX;
        int minY;
        int maxX;
        int maxY;

        bool operator==(const CellAABB &other) const {
            return (other.minX == minX && other.minY == minY &&
                    other.maxX == maxX && other.maxY == maxY);
        }

        
    };
    

    std::unordered_map<pair<int,int>,vector<QBody*>,PairHash> cells;

    static bool SortBodiesHorizontal(const QBody *bodyA,const QBody *bodyB){
        if(bodyA->GetAABB().GetMin().x==bodyB->GetAABB().GetMin().x){
            return bodyA->GetAABB().GetMax().y>bodyB->GetAABB().GetMax().y;
        }
        return bodyA->GetAABB().GetMin().x<bodyB->GetAABB().GetMin().x;
    };

    std::unordered_map<QBody*,CellAABB> bodyOldCells;

public:
    QSpatialHashing(vector<QBody*>& worldBodies, float sizeOfCells=128.0f);

    void Clear();

    void Update();

    void GetAllPairs(std::unordered_set<std::pair<QBody*, QBody*>,QBody::BodyPairHash,QBody::BodyPairEqual> &pairs);    

    
};


#endif // QSPATIALHASHING_H