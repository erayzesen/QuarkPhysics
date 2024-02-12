
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

template <typename T>
struct QObjectPoolNode
{
    public:
    QObjectPoolNode(int nextIndex) : next(nextIndex) {
        data = new T();
    }

    QObjectPoolNode() {
        data=new T();
    }

    ~QObjectPoolNode() {
        delete data;
    }

    T* data;
    int next;
    int id;
};

template <typename T>
class QObjectPool {
    QObjectPoolNode<T>* nodes;
    size_t size = 2;
    size_t freeSize = 2;
    size_t lastFreeNode = -1;

    void Resize(size_t newSize);

public:
    QObjectPool();
    QObjectPool(size_t initialSize);
    ~QObjectPool();

    QObjectPoolNode<T> * Create();
    void Free(int nodeID);
    void FreeAll();
    void Reset();
};

#endif // QOBJECTPOOL_H


