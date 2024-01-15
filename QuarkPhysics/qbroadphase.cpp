
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
#include "qworld.h"

QBroadPhase::QBroadPhase(QWorld * targetWorld)
{
    world=targetWorld;
}

QBroadPhase::~QBroadPhase()
{
    ClearTree();
}

void QBroadPhase::ReCreateTree(vector<QBody *> &objectList)
{

    ClearTree();
    if(objectList.size()==0 ) return;

    for(auto object:objectList){
        QAABB fattedAABB=object->GetAABB().FattedWithRate(1.2f);
        Node *newNode=new Node(object, fattedAABB, GetNewNodeID() );
        newNode->isLeaf=true;
        nodes.push_back ( newNode );
    }

    

    while(nodes.size()>1  ){

        int bestIndexA=-1, bestIndexB=-1;
        QAABB bestMergedAABB;
        float bestSAH= std::numeric_limits<float>::infinity();


        for (int i=0;i<nodes.size();++i ){

            for (int j=i+1;j<nodes.size();++j ){

                

                QAABB mergedAABB=QAABB::Combine (nodes[i]->aabb,nodes[j]->aabb);
                float sah=mergedAABB.GetArea();

                if(sah<bestSAH){
                    bestSAH=sah;
                    bestIndexA=i;
                    bestIndexB=j;
                    bestMergedAABB=mergedAABB;
                }
                
            }

        }

        Node *mergedNode=new Node(nullptr,bestMergedAABB, GetNewNodeID());

        mergedNode->isLeaf=false;
        mergedNode->left=nodes[bestIndexA];
        mergedNode->right=nodes[bestIndexB];

        mergedNode->left->parent=mergedNode;
        mergedNode->right->parent=mergedNode;

        if(bestIndexA>bestIndexB){
            nodes.erase(nodes.begin()+bestIndexA );
            nodes.erase(nodes.begin()+bestIndexB );
        }else{
            nodes.erase(nodes.begin()+bestIndexB );
            nodes.erase(nodes.begin()+bestIndexA );
        }

        
        nodes.push_back(mergedNode);

    }

    root=nodes[0];
    


}

void QBroadPhase::ClearTree()
{
    for (auto node : nodes){
        delete node;
    }
    nodes.clear();
    lastID=0;
}

void QBroadPhase::Query(QAABB &AABB, vector<QBody*> &list)
{
    QueryInNode(root,AABB,list);

}

void QBroadPhase::QueryInNode(Node *node, QAABB &AABB, vector<QBody *> &list)
{
    if(node==nullptr){
        cout<<"Error: BroadPhase::QueryInNode - An invalid node! ";
        return;
    }
    world->debugAABBTestCount+=1;
    if(node->isLeaf==true){
        if(node->obj->GetAABB().isCollidingWith(AABB)==true )
            list.push_back(node->obj);
        return;
    }

    if(node->aabb.isCollidingWith(AABB)==false )
        return;
    

    QueryInNode(node->left,AABB,list);
    QueryInNode(node->right,AABB,list);
    

}

void QBroadPhase::Insert(QBody *object)
{
}

void QBroadPhase::Remove(QBody *object)
{
}

void QBroadPhase::Update(QBody *object, QAABB &AABB)
{
}

int QBroadPhase::GetNewNodeID()
{
    int res=lastID;
    lastID+=1;
    return res;
}

void QBroadPhase::GetPairs(vector<pair<QBody *, QBody *>> &pairCollection)
{
    if(root==nullptr || root->isLeaf){
        return;
    }

    unordered_set<int> checkedHashes;

    GetPairsBetweenNodes(root->left,root->right,pairCollection,checkedHashes);
}

void QBroadPhase::GetPairsBetweenNodes(Node * nodeA, Node *nodeB, vector<pair<QBody *, QBody *>> &pairCollection,unordered_set<int> &checkedPairList)
{
    int pairHash=(nodeA->id + nodeB->id)*(nodeA->id + nodeB->id+1)/2+nodeB->id;
    if(checkedPairList.find(pairHash)!=checkedPairList.end() )
        return;

    checkedPairList.insert(pairHash);


    if(nodeA->isLeaf && nodeB->isLeaf){
        if(nodeA->obj->GetAABB().isCollidingWith(nodeB->obj->GetAABB() ) ){
            pairCollection.push_back(pair<QBody*,QBody*> (nodeA->obj,nodeB->obj) );
        }
    }else if(!nodeA->isLeaf && !nodeB->isLeaf){
        GetPairsBetweenNodes(nodeA->left,nodeA->right,pairCollection,checkedPairList);
        GetPairsBetweenNodes(nodeB->left,nodeB->right,pairCollection,checkedPairList);

        if(nodeA->aabb.isCollidingWith(nodeB->aabb) ){
            GetPairsBetweenNodes(nodeA->left,nodeB->left,pairCollection,checkedPairList);
            GetPairsBetweenNodes(nodeA->right,nodeB->right,pairCollection,checkedPairList);

            GetPairsBetweenNodes(nodeA->left,nodeB->right,pairCollection,checkedPairList);
            GetPairsBetweenNodes(nodeA->right,nodeB->left,pairCollection,checkedPairList);
        }
        
    }else if(nodeA->isLeaf && !nodeB->isLeaf){
        GetPairsBetweenNodes(nodeB->left,nodeB->right,pairCollection,checkedPairList);
        if(nodeA->aabb.isCollidingWith(nodeB->aabb) ){
            GetPairsBetweenNodes(nodeA ,nodeB->left,pairCollection,checkedPairList);
            GetPairsBetweenNodes(nodeA ,nodeB->right,pairCollection,checkedPairList);
        }
    }else if(!nodeA->isLeaf && nodeB->isLeaf){
        GetPairsBetweenNodes(nodeA->left,nodeA->right,pairCollection,checkedPairList);
        if(nodeB->aabb.isCollidingWith(nodeA->aabb) ){
            GetPairsBetweenNodes(nodeB ,nodeA->left,pairCollection,checkedPairList);
            GetPairsBetweenNodes(nodeB ,nodeA->right,pairCollection,checkedPairList);
        }
    }


}
