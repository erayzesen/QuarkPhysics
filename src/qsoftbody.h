#ifndef QSOFTBODY_H
#define QSOFTBODY_H
#include "qbody.h"

class QSoftBody : public QBody
{
	//#Softbody Properties
	float rigidity=1.0f;
	float volumePreserving=false;
	float volumeRate=0.8f;
	float volumeRigidity=1.0f;
	float targetVolumeArea=0.0f;

	float particularMass=1.0f;
	bool useParticularMass=false;


	float circumference=0.0f;
	bool volumeStability=false;
	bool passivationOfInternalSprings=false;
	bool selfParticleCollision=false;

	bool shapeMatching=false;
	float shapeMatchingRate=0.4f;

public:
	QSoftBody();



	//Properties Set Methods (it returns this)
	QSoftBody * SetRigidity(float val){
		rigidity=val;
		return this;
	};
	QSoftBody * SetVolumeRate(float val){
		volumeRate=val;
		return this;
	}
	QSoftBody * SetVolumeRigidity(float val){
		volumeRigidity=val;
		return this;
	}
	QSoftBody * SetVolumePreserving(bool val){
		volumePreserving=val;
		if(val==true){
			targetVolumeArea=GetTotalPolygonsInitialArea();
		}
		return this;
	}

	QSoftBody *SetTargetVolumeArea(float val){
		targetVolumeArea=val;
		return this;
	}
	QSoftBody *SetSelfParticleCollision(bool val){
		selfParticleCollision=val;
		return this;
	}

	QSoftBody *SetPassivationOfInternalSprings(bool val){
		passivationOfInternalSprings=val;
		return this;
	}

	QSoftBody *SetShapeMatching(bool val){
		shapeMatching=val;
		return this;
	}

	QSoftBody *SetShapeMatchingRate(float val){
		shapeMatchingRate=val;
		return this;
	}
	QSoftBody *SetParticularMass(float val){
		particularMass=val;
		return this;
	}

	QSoftBody *SetUseParticularMass(bool val){
		useParticularMass=val;
		return this;
	}

	//Properties Get Methods
	float GetMass(){
		if(useParticularMass)
			return particularMass;
		else
			return mass;
	}
	float  GetRigidity(){
		return rigidity;
	};
	float  GetVolumeRate(){
		return volumeRate;
	};
	float  GetVolumeRigidity(){
		return volumeRigidity;
	};
	bool GetVolumePreserving(){
		return volumePreserving;
	}
	float GetTargetVolumeArea(){
		return targetVolumeArea;
	}
	bool GetSelfParticleCollision(){
		return selfParticleCollision;
	}
	bool GetPassivationOfInternalSprings(){
		return passivationOfInternalSprings;
	}
	bool GetShapeMatching(){
		return shapeMatching;
	}

	float GetShapeMatchingRate(){
		return shapeMatchingRate;
	}

	float GetParticularMass(){
		return particularMass;
	}
	bool GetUseParticularMass(){
		return useParticularMass;
	}

	//Helper methods

	float safe_asin(float value){
		if(value<-1.0f){
			return asin(-1.0);
		}else if(value>1.0f){
			return asin(1.0);
		}else{
			return asin(value);
		}
	}



	//
	void Update();
	void PreserveVolumes();
	void ApplyShapeMatching();

};

#endif // QSOFTBODY_H
