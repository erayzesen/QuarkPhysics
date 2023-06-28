#include "qsoftbody.h"
#include "qworld.h"
#include "qmesh.h"
#include "cmath"

QSoftBody::QSoftBody()
{
	simulationModel=SimulationModels::MASS_SPRING;
}






void QSoftBody::Update()
{
	//Integrate Velocities
	for(int i=0;i<_meshes.size();i++){
		QMesh *mesh=_meshes[i];
		for(int n=0;n<mesh->GetParticleCount();n++){
			QParticle *particle=mesh->GetParticle(n);
			if(GetVolumePreserving() && particle->GetIsInternal())
				continue;
			auto vel=particle->GetGlobalPosition()-particle->GetPreviousGlobalPosition();
			particle->SetPreviousGlobalPosition(particle->GetGlobalPosition() );
			particle->ApplyForce(vel);
			particle->ApplyForce(mass*world->GetGravity());
			particle->ApplyForce(particle->GetForce());
			particle->SetForce(QVector::Zero());
		}
	}

	if(volumePreserving){
		PreserveVolumes();
	}

	if(shapeMatching){
		ApplyShapeMatching();
	}



	UpdateAABB();


}

void QSoftBody::PreserveVolumes()
{

	for(auto mesh:_meshes){
		if(mesh->GetSpringCount()==0)continue;
		float currentMeshesArea=mesh->GetPolygonsArea();
		float circumference=GetCircumference();

		float deltaArea=(targetVolumeArea*volumeRate)-currentMeshesArea;
		if(volumeStability==false){
			if(deltaArea<0){
				deltaArea=0;
			}else{
				volumeStability=true;
			}
		}


		float pressure=(deltaArea/circumference)*volumeRigidity;
		QVector volumeForces[mesh->GetSpringCount()];

		for(int i=0;i<mesh->GetSpringCount();i++){
			QSpring *spring=mesh->GetSpringAt(i);
			if(spring->GetIsInternal()==true)continue;
			QVector vec=spring->GetParticleB()->GetGlobalPosition()-spring->GetParticleA()->GetGlobalPosition();

			volumeForces[i]=pressure*vec.Normalized().Perpendicular();
		}
		for(int i=0;i<mesh->GetSpringCount();i++){
			QSpring *spring=mesh->GetSpringAt(i);
			if(spring->GetIsInternal()==true)continue;
			QVector centerPos=(spring->GetParticleB()->GetGlobalPosition()+spring->GetParticleA()->GetGlobalPosition())*0.5f;
			QParticle::ApplyForceToParticleSegment(spring->GetParticleA(),spring->GetParticleB(),volumeForces[i],centerPos);


		}


	}

}

void QSoftBody::ApplyShapeMatching()
{
	for(auto mesh:_meshes){
		if(mesh->GetParticleCount()<2)
			continue;
		QVector avaragePosition=QVector::Zero();
		vector<QVector> localParticlePositions;
		for(int i=0;i<mesh->GetParticleCount();i++){
			QParticle *particle=mesh->GetParticle(i);
			avaragePosition+=particle->GetGlobalPosition();
			localParticlePositions.push_back(particle->GetPosition());

		}
		avaragePosition/=mesh->GetParticleCount();
		float avarageRotation=0;
		float avarageCosA=0.0f;
		float avarageSinA=0.0f;
		int n=0;
		for(int i=0;i<mesh->GetParticleCount();i++){
			QParticle *particle=mesh->GetParticle(i);
			QVector originalPos=localParticlePositions[i];
			QVector relativePos=particle->GetGlobalPosition()-avaragePosition;
			//if((relativePos-originalPos).Length()==0.0f) continue;
			float dotLength=relativePos.Length()*localParticlePositions[i].Length();
			float dot=relativePos.Dot(originalPos);
			float perpDot=relativePos.Dot(originalPos.Perpendicular());
			avarageCosA+=dotLength==0.0f ? 0.0f:dot/dotLength;


			avarageSinA+=dotLength==0.0f ? 0.0f:perpDot/dotLength;

			++n;


		}
		if(n==0)return;

		avarageCosA/=n;
		avarageSinA/=n;
		float aSin=safe_asin(avarageSinA);
		float rad=atan2(aSin,avarageCosA);
		avarageRotation=rad;
		//cout<<avarageRotation/(M_PI/180)<<endl;


		for(int i=0;i<localParticlePositions.size();i++){
			QVector targetPos=localParticlePositions[i].Rotated(-avarageRotation);
			targetPos+=avaragePosition;
			//world->GetGizmos()->push_back(new QGizmoCircle(targetPos,3.0f) );
			QVector distance=targetPos-mesh->GetParticle(i)->GetGlobalPosition();
			QVector distanceUnit=distance.Normalized();
			float distanceLen=distance.Length();
			QVector force=(distanceLen*distanceLen*distanceUnit)*shapeMatchingRate*0.01f;
			mesh->GetParticle(i)->ApplyForce(force);
		}

	}
}



