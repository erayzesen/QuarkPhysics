
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
protected:
    std::unordered_set<pair<QBody*, QBody*>,QBody::BodyPairHash,QBody::BodyPairEqual> pairs;
    bool BodiesCanCollide(QBody * bodyA,QBody *bodyB){
        return QBody::CanCollide(bodyA,bodyB);
    }

public:
    vector<QBody*> &bodies;
    QBroadPhase(vector<QBody*> &worldBodies): bodies(worldBodies){};
    ~QBroadPhase(){};


    virtual void Clear();

    virtual std::unordered_set<pair<QBody*, QBody*>,QBody::BodyPairHash,QBody::BodyPairEqual> &GetPairs();

    virtual void Insert(QBody* body);

    virtual void Remove(QBody* body);

	 
};


#endif // QBROADPHASE_H