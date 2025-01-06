
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

#include "qaabb.h"
#include "qparticle.h"

QAABB QAABB::GetAABBFromParticles(vector<QParticle*> &particleCollection)
{
    float minX=9999999.0;
    float minY=9999999.0;
    float maxX=-9999999.0;
    float maxY=-9999999.0;

    for(int n=0;n<particleCollection.size();n++){
        QParticle *particle=particleCollection[n];
        float  r=particle->GetRadius()>0.5f ? particle->GetRadius():0;
        float pMinX=particle->GetGlobalPosition().x-r;
        float pMinY=particle->GetGlobalPosition().y-r;
        float pMaxX=particle->GetGlobalPosition().x+r;
        float pMaxY=particle->GetGlobalPosition().y+r;

        if(pMinX<minX)
            minX=pMinX;
        if(pMinY<minY)
            minY=pMinY;
        if(pMaxX>maxX)
            maxX=pMaxX;
        if(pMaxY>maxY)
            maxY=pMaxY;
    }
    


    return QAABB(QVector(minX,minY),QVector(maxX,maxY) );
}
