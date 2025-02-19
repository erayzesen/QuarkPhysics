
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

#include "qangleconstraint.h"
#include <iostream>
#include "qmesh.h"
#include "qbody.h"

float QAngleConstraint::GetAngleOfParticlesWithLocalPositions(QParticle *particleA,QParticle *particleB,QParticle *particleC)
{
	QVector toPrev=particleA->GetPosition()-particleB->GetPosition();
	QVector toNext=particleC->GetPosition()-particleB->GetPosition();

	float cosA=toNext.Dot(toPrev)/(toPrev.Length()*toNext.Length() );
	float sinA=toNext.Dot(toPrev.Perpendicular() )/(toPrev.Length()*toNext.Length() );

	float angleRad=atan2(sinA,cosA);

	if(angleRad<0){
		angleRad=(M_PI*2.0)-abs(angleRad);
	}
	return angleRad;
}



QAngleConstraint::QAngleConstraint(QParticle *particleA, QParticle *particleB, QParticle *particleC,float angleRange)
{
	pA=particleA;
	pB=particleB;
	pC=particleC;

	if(pA!=nullptr || pB!=nullptr || pC!=nullptr){
		currentAngle=GetAngleOfParticlesWithLocalPositions(particleA,particleB,particleC);
		minAngle=currentAngle-angleRange;
		maxAngle=currentAngle+angleRange;

	}

}

QAngleConstraint::QAngleConstraint(QParticle *particleA, QParticle *particleB, QParticle *particleC, float minimumAngle, float maximumAngle)
{
	pA=particleA;
	pB=particleB;
	pC=particleC;
	minAngle=minimumAngle;
	maxAngle=maximumAngle;

}

void QAngleConstraint::Update(float specifiedRigidity,bool addToAccumulatedForces)
{
	if(enabled==false)
		return;

	float rigidityFactor=(specifiedRigidity>=0.0f && specifiedRigidity<=1.0f) ? specifiedRigidity:rigidity; 

	QVector toPrev=pA->GetGlobalPosition()-pB->GetGlobalPosition();
	QVector toNext=pC->GetGlobalPosition()-pB->GetGlobalPosition();

	float cosA=toNext.Dot(toPrev)/(toPrev.Length()*toNext.Length() );
	float sinA=toNext.Dot(toPrev.Perpendicular() )/(toPrev.Length()*toNext.Length() );

	float angleRad=atan2(sinA,cosA);

	if(angleRad<0){
		angleRad=(M_PI*2.0)-abs(angleRad);
	}

	if(beginToSaveAngles){
		prevAngle=angleRad;
		currentAngle=angleRad;
		beginToSaveAngles=false;
		return;
	}
	QVector d1=QVector::AngleToUnitVector(prevAngle);
	QVector d2=QVector::AngleToUnitVector(angleRad);
	float angleDifference=QVector::AngleBetweenTwoVectors(d2,d1);

	

	angleRad=prevAngle+angleDifference;

	currentAngle=angleRad;

	if(angleRad>maxAngle){
		float diffAngle=maxAngle-angleRad;
		float angularForce=diffAngle*0.5f;

		
		if (pA->GetEnabled() ){
			QVector targetPosition=pB->GetGlobalPosition()+toPrev.Rotated(angularForce);
			QVector force=(targetPosition-pA->GetGlobalPosition() )*rigidityFactor;
			if(addToAccumulatedForces){
				pA->AddAccumulatedForce( force );
			}else{
				pA->ApplyForce( force );
			}
		}
		if (pC->GetEnabled() ){
			QVector targetPosition=pB->GetGlobalPosition()+toNext.Rotated(-angularForce);
			QVector force=(targetPosition-pC->GetGlobalPosition() )*rigidityFactor;
			if(addToAccumulatedForces){
				pC->AddAccumulatedForce(force );
			}else{
				pC->ApplyForce(force);
			}
		}

		
		
	}

	if(angleRad<minAngle){
		float diffAngle=minAngle-angleRad;
		float angularForce=diffAngle*0.5f;

		if (pA->GetEnabled() ){
			QVector targetPosition=pB->GetGlobalPosition()+toPrev.Rotated(angularForce);
			QVector force=(targetPosition-pA->GetGlobalPosition() )*rigidityFactor;
			if(addToAccumulatedForces){
				pA->AddAccumulatedForce(force );
			}else{
				pA->ApplyForce( force );
			}
		}
		if (pC->GetEnabled() ){
			QVector targetPosition=pB->GetGlobalPosition()+toNext.Rotated(-angularForce);
			QVector force=(targetPosition-pC->GetGlobalPosition() )*rigidityFactor;
			if(addToAccumulatedForces){
				pC->AddAccumulatedForce(force );
			}else{
				pC->ApplyForce( force );
			}
		}

		
	}
	prevAngle=angleRad;


}
