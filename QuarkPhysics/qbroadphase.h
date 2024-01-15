
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
#include "qaabb.h"




class QBroadPhase {   
public:
    QWorld *world;
    QBroadPhase(QWorld * targetWorld);
    QBroadPhase(){};
    ~QBroadPhase();


    
    struct Node{
        QAABB aabb;
        QBody *obj;
        Node *left;
        Node *right;
        bool isLeaf;
        Node *parent;
        int id;
        Node(QBody *object, QAABB AABB, int nodeID) : obj(object), aabb(AABB) {
            isLeaf=true;
            id=nodeID;
        }; 
        
        
    };

    Node *root;
    vector<Node*> nodes;

    void ReCreateTree(vector<QBody *> &objectList);
    void ClearTree();

    void Query(QAABB &AABB, vector<QBody*> &list);

    void QueryInNode(Node* node, QAABB &AABB, vector<QBody*> &list);

    void GetPairs(vector<pair<QBody*,QBody*> > &pairCollection);
    void GetPairsBetweenNodes(Node * nodeA, Node *nodeB, vector<pair<QBody*,QBody*> > &pairCollection, unordered_set<int> &checkedPairList);

    void Insert(QBody *object);
    void Remove(QBody *object);
    void Update(QBody *object, QAABB &AABB);

    int GetNewNodeID();

    int lastID=0;

    
    

	 
};


#endif // QBROADPHASE_H
