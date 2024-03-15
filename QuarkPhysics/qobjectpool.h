
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

// qobjectpool.h
#ifndef QOBJECTPOOL_H
#define QOBJECTPOOL_H

#include <stdlib.h>
#include <vector>
#include <iostream>

using namespace std;

template <typename T>
class QObjectPool {
public:
    struct Node
    {
        public:

        Node() {
            
        }

        

        T* data;
        int next;
        int id;
        
    };
private:

    /* Node* nodes;
    size_t size = 2;
    size_t freeSize = 2;
    size_t lastFreeNode = -1; */

    void Resize(size_t newSize){
        QObjectPool<T>::Node* newNodes = new QObjectPool<T>::Node[newSize];

        if(size>0){
            for (size_t i = 0; i < std::min(size, newSize); ++i) {
                newNodes[i] = nodes[i];
            }
            delete[] nodes;
        }

        for (size_t i = size; i < newSize; ++i) {
            newNodes[i].next = i + 1;
            newNodes[i].id = i;
            newNodes[i].data = new T();
        }
        
        
        nodes = newNodes;
        size = newSize;
    };

public:
    QObjectPool(){
        nodes = new QObjectPool<T>::Node[size];

        for (size_t i = 0; i < size; ++i) {
            nodes[i].next = i + 1;
            nodes[i].id = i;
            nodes[i].data=new T();
        }

        nodes[size - 1].next = -1;
        lastFreeNode = 0;
    };
    QObjectPool(size_t initialSize,size_t expansionSize=0){
        size=initialSize;
        sizeStep=expansionSize;
        nodes = new QObjectPool<T>::Node[size];

        for (size_t i = 0; i < size; ++i) {
            nodes[i].next = i + 1;
            nodes[i].id = i;
            nodes[i].data=new T();
        }

        lastFreeNode = 0;
    };
    ~QObjectPool(){
        ClearAll();
    };

    Node* nodes;
    size_t size = 2;
    size_t sizeStep = 0;
    size_t freeSize = 2;
    size_t lastFreeNode = 0;
    

    Node & Create(){
        //cout<<"pool size: "<<size<<" free node index: "<< lastFreeNode<<endl;
        if (lastFreeNode==size ) {
            if (sizeStep==0){
                if(size==0){
                    Resize(2);
                }else{
                    Resize(size * 2);
                }
                
            }else{
                Resize(size+sizeStep);
            }
            
        }

        size_t newNodeIndex=lastFreeNode;
        lastFreeNode = nodes[lastFreeNode].next;

        return nodes[newNodeIndex];
    };
    void Free(int nodeID){
        nodes[nodeID].next = lastFreeNode;
        lastFreeNode = nodeID;
    };
    void FreeAll(){
        for (size_t i = 0; i < size; i++) {
            nodes[i].next = i + 1;
        }

        lastFreeNode = 0;
    };
    void ClearAll(){
        if(size>0){
            for (int i=0;i<size;++i){
                //cout<<"pool object deleted index:"<<i<<endl;
                delete nodes[i].data;
                nodes[i].data=nullptr;
            }
            delete[] nodes;
            size=0;
            lastFreeNode=0;
        }
        
    };

    
};



#endif // QOBJECTPOOL_H


