
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

// qobjectpool.cpp
#include "qobjectpool.h"
#include <iostream>
#include <algorithm>

template <typename T>
void QObjectPool<T>::Resize(size_t newSize) {
    QObjectPoolNode<T>* newNodes = new QObjectPoolNode<T>[newSize];

    for (size_t i = 0; i < std::min(size, newSize); ++i) {
        newNodes[i] = nodes[i];
    }

    for (size_t i = size - 1; i < newSize; ++i) {
        newNodes[i]=new QObjectPoolNode();
        newNodes[i].next = i + 1;
        newNodes[i].id = i;
    }

    if (lastFreeNode == -1) {
        lastFreeNode = size;
    }

    delete[] nodes;
    nodes = newNodes;
    size = newSize;
    
    
}







template <typename T>
QObjectPool<T>::QObjectPool() {
    nodes = new QObjectPoolNode<T>[size];

    for (size_t i = 0; i < size - 1; i++) {
        nodes[i]=new QObjectPoolNode<T>();
        nodes[i].next = i + 1;
        nodes[i].id = i;
    }

    nodes[size - 1].next = -1;
    lastFreeNode = 0;
}

template <typename T>
QObjectPool<T>::QObjectPool(size_t initialSize) {
    size = initialSize;
    nodes = new QObjectPoolNode<T>[size];

    QObjectPool();
}

template <typename T>
QObjectPool<T>::~QObjectPool() {
    delete[] nodes;
}

template <typename T>
QObjectPoolNode<T> * QObjectPool<T>::Create() {
    if (lastFreeNode == -1) {
        Resize(size * 2);
    }

    size_t newNodeIndex=lastFreeNode;
    lastFreeNode = nodes[lastFreeNode].next;

    return nodes[newNodeIndex];

}

template <typename T>
void QObjectPool<T>::Free(int nodeID) {
    nodes[nodeID].next = lastFreeNode;
    lastFreeNode = nodeID;
}

template <typename T>
void QObjectPool<T>::FreeAll() {
    for (size_t i = 0; i < size - 1; i++) {
        nodes[i].next = i + 1;
    }

    nodes[size - 1].next = -1;
    lastFreeNode = 0;
}

template <typename T>
void QObjectPool<T>::Reset() {
    delete[] nodes;
}
