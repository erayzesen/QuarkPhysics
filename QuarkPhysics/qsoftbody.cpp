
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

#include "qsoftbody.h"
#include "qparticle.h"
#include "qvector.h"
#include "qworld.h"
#include "qmesh.h"
#include "cmath"
#include <utility>

QSoftBody::QSoftBody()
{
	simulationModel=SimulationModels::MASS_SPRING;
	bodyType=QBody::BodyTypes::SOFT;
}






void QSoftBody::Update()
{
	if (mode==QBody::STATIC){
		return;
	}

	//Sleep Feature

	if(isSleeping){
		return;
	}

	//Time scale feature
	
	float ts=1.0f;

	if(enableBodySpecificTimeScale==true){
		ts=bodySpecificTimeScale;
	}else{
		if(world!=nullptr){
			ts=world->GetTimeScale();
		}
	}

	//Integrate Velocities
	for(int i=0;i<_meshes.size();i++){
		QMesh *mesh=_meshes[i];
		for(int n=0;n<mesh->GetParticleCount();n++){
			QParticle *particle=mesh->GetParticleAt(n);
			if(GetPassivationOfInternalSpringsEnabled() && particle->GetIsInternal())
				continue;
			auto vel=particle->GetGlobalPosition()-particle->GetPreviousGlobalPosition();
			particle->SetPreviousGlobalPosition(particle->GetGlobalPosition() );
			particle->ApplyForce(vel-(vel*airFriction) );
			particle->ApplyForce(mass*world->GetGravity()*ts);
			particle->ApplyForce(particle->GetForce());
			particle->SetForce(QVector::Zero());
		}
	}


	if(enableAreaPreserving){
		PreserveAreas();
	}

	//It's added to world contraints operations
	/* if(enableShapeMatching){
		ApplyShapeMatching();
	} */



	UpdateAABB();


}

void QSoftBody::PreserveAreas()
{
	//Time scale feature
	float ts=1.0f;

	if(enableBodySpecificTimeScale==true){
		ts=bodySpecificTimeScale;
	}else{
		if(world!=nullptr){
			ts=world->GetTimeScale();
		}
	}

	for(auto mesh:_meshes){

		if(mesh->GetSpringCount()==0)continue;
		float currentMeshesArea=mesh->GetPolygonsArea();
		float circumference=GetCircumference();

		float deltaArea=(targetPreservationArea*areaPreservingRate)-currentMeshesArea;
		if(enableAreaStability==false){
			if(deltaArea<0){
				deltaArea=0;
			}else{
				enableAreaStability=true;
			}
		}


		float pressure=(deltaArea/circumference)*areaPreservingRigidity;

		for(int i=0;i<mesh->GetClosedPolygonCount();i++){
			vector<QParticle*> &polygon=mesh->GetClosedPolygonAt(i);
			QVector volumeForces[polygon.size()];
			for(int n=0;n<polygon.size();n++){
				QSpring *spring=mesh->GetSpringAt(i);
				QParticle *pp=polygon[ (n-1+polygon.size())%polygon.size() ];
				QParticle *np=polygon[ (n+1)%polygon.size() ];
				QVector vec=np->GetGlobalPosition()-pp->GetGlobalPosition();
				volumeForces[n]=pressure*(vec.Perpendicular().Normalized())*ts;	
			}

			for(int n=0;n<polygon.size();n++){
				QParticle *pp=polygon[ (n-1+polygon.size())%polygon.size() ];
				QParticle *np=polygon[ (n+1)%polygon.size() ];
				QVector centerPos=(np->GetGlobalPosition()+pp->GetGlobalPosition())*0.5f;
				QParticle::ApplyForceToParticleSegment(pp,np,volumeForces[n],centerPos);
				//GetWorld()->GetGizmos()->push_back( new QGizmoLine( centerPos,centerPos+volumeForces[n]*30,true ) );

			}
		}


	}

}



pair<QVector, float> QSoftBody::GetAveragePositionAndRotation(vector<QParticle*> particles){
	if(particles.size()==1)
		return pair<QVector, float>(particles[0]->GetGlobalPosition(),0.0f);
	//Finding Actual Position
	QVector averagePosition=QVector::Zero();
	QVector localCenterPosition=QVector::Zero();
	for(int i=0;i<particles.size();i++){
		QParticle *particle=particles[i];
		averagePosition+=particle->GetGlobalPosition();
		localCenterPosition+=particle->GetPosition();

	}  
	averagePosition/=particles.size();
	localCenterPosition/=particles.size();
	float averageRotation=0;
	float averageCosA=0.0f;
	float averageSinA=0.0f;
	for(int i=0;i<particles.size();i++){
		QParticle *particle=particles[i];
		QVector originalPos=particle->GetPosition()-localCenterPosition;
		QVector relativePos=particle->GetGlobalPosition()-averagePosition;
		float dotLength=relativePos.Length()*originalPos.Length();
		float dot=relativePos.Dot(originalPos);
		float perpDot=relativePos.Dot(originalPos.Perpendicular());
		averageCosA+=dotLength==0.0f ? 0.0f:dot/dotLength;
		averageSinA+=dotLength==0.0f ? 0.0f:perpDot/dotLength;
	}

	averageCosA/=particles.size();
	averageSinA/=particles.size();
	float aSin=safe_asin(averageSinA);
	float rad=atan2(aSin,averageCosA);
	averageRotation=rad;

	//averagePosition-localCenterPosition;
	

	return pair< QVector, float >(averagePosition,averageRotation);

}


void QSoftBody::ApplyShapeMatching()
{
	//Time scale feature
	float ts=1.0f;

	if(enableBodySpecificTimeScale==true){
		ts=bodySpecificTimeScale;
	}else{
		if(world!=nullptr){
			ts=world->GetTimeScale();
		}
	}

	for(int i=0;i<_meshes.size();i++){
		QMesh *mesh=_meshes[i];
		if(mesh->GetParticleCount()<2)
			continue;

		vector<QParticle*> particles;
		if(mesh->GetCollisionBehavior()==QMesh::CollisionBehaviors::POLYLINE ){
			for(int n=0;n<mesh->closedPolygons.size();n++ ){
				vector<QParticle*> polygon=mesh->closedPolygons[n];
				particles.insert(particles.end(),polygon.begin(),polygon.end() );
			}
		}else{
			particles=mesh->particles;
		}

		QVector localCenterPosition;
		for(auto particle:particles){
			localCenterPosition+=particle->GetPosition();
		}
		localCenterPosition/=particles.size();
		
		QVector averagePosition;
		float averageRotation;
		if(enableShapeMatchingFixedTransform){
			localCenterPosition=QVector::Zero();
			averagePosition=shapeMatchingFixedPosition;
			averageRotation=shapeMatchingFixedRotation;
		}else{
			auto averagePositionAndRotation=GetAveragePositionAndRotation(particles);
			averagePosition=averagePositionAndRotation.first;
			averageRotation=averagePositionAndRotation.second;

		}	




		for(int n=0;n<particles.size();n++){
			QParticle * particle=particles[n];
			QVector targetPos=(particle->GetPosition()-localCenterPosition).Rotated(-averageRotation);
			targetPos+=averagePosition;
			//world->GetGizmos()->push_back(new QGizmoCircle(targetPos,3.0f) );
			QVector distance=targetPos-particle->GetGlobalPosition();
			QVector distanceUnit=distance.Normalized();
			float distanceLen=distance.Length();
			float forceLinear=min(distanceLen*distanceLen*0.002f*shapeMatchingRate*ts,distanceLen*ts);
			QVector force=forceLinear*distanceUnit;
			particle->ApplyForce(force);
		}

	}
}





