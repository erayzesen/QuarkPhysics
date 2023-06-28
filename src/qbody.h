#ifndef QBODY_H
#define QBODY_H

#include "qvector.h"
#include "qaabb.h"
#include <vector>
#include "qmesh.h"


class QWorld;

class QBody{
	float interia=0.0f;
	float circumference=0.0f;
public:
	enum Modes{
		DYNAMIC,
		STATIC
	};

	enum SimulationModels{
		MASS_SPRING,
		RIGID_BODY
	};

protected:

	//General Properties

	QWorld *world;
	QVector position=QVector(0,0);
	QVector prevPosition=QVector::Zero();
	float rotation=0.0f;
	float prevRotation=0.0f;
	QAABB aabb;
	QAABB fattedAABB;
	Modes mode=QBody::Modes::DYNAMIC;
	bool interiaNeedsUpdate=true;
	bool circumferenceNeedsUpdate=true;

	//Material Properties;

	float friction=0.2f;
	float staticFriction=0.5f;
	float mass=1.0f;
	float restitution=0.0f;

	//Collision Features
	int layersBit=1;
	int collidableLayersBit=1;

	//Sleeping Features
	bool isSleeping=false;

	int sleepTick=120;
	int fixedVelocityTick=0;
	int fixedAngularTick=0;
	//Island Features
	bool visited=false;
	bool canSleep=true;

	void UpdateAABB();
	void UpdateMeshTransforms();
	virtual void Update(){};
	bool CanGiveCollisionResponseTo(QBody *otherBody);

	public:
		QBody();
		virtual ~QBody();




		//General Get Methods

		QWorld *GetWorld(){
			return world;
		}
		QVector GetPosition(){
			return position;
		}
		QVector GetPreviousPosition(){
			return prevPosition;
		}
		float GetRotation(){
			return rotation;
		}
		float GetPreviousRotation(){
			return prevRotation;
		}
		QAABB GetAABB()const{
			return aabb;
		}
		QAABB GetFattedAABB()const{
			return fattedAABB;
		}
		float GetTotalInitialArea(){
			float res=0.0f;
			for(auto mesh:_meshes){
				res+=mesh->GetInitialArea();
			}
			return res;
		}
		float GetTotalPolygonsInitialArea(){
			float res=0.0f;
			for(auto mesh:_meshes){
				res+=mesh->GetInitialPolygonsArea();
			}
			return res;
		}
		float GetTotalArea(){
			float res=0.0f;
			for(auto mesh:_meshes){
				res+=mesh->GetArea();
			}
			return res;
		}
		float GetTotalPolygonsArea(){
			float res=0.0f;
			for(auto mesh:_meshes){
				res+=mesh->GetPolygonsArea();
			}
			return res;
		}
		Modes GetMode(){
			return mode;
		}
		float GetInteria(){
			if(interiaNeedsUpdate==true){
				interia=GetTotalInitialArea()*2.0f*mass;
				interia=interia==0.0f ? 0.25:interia;
				interiaNeedsUpdate=false;
			}
			return interia;
		}
		int GetLayersBit(){
			return layersBit;
		}
		int GetCollidableLayersBit(){
			return collidableLayersBit;
		}
		bool GetOverlapWithCollidableLayersBit(int layersBit){
			if( (layersBit & this->collidableLayersBit)==0 )
				return false;
			return true;
		}
		bool GetOverlapWithLayersBit(int layersBit){
			if( (layersBit & this->layersBit)==0 ){
				return false;
			}
			return true;
		}
		bool GetIsSleeping(){
			return isSleeping;
		}
		bool GetCanSleep(){
			return canSleep;
		}
		SimulationModels GetSimulationModel(){
			return simulationModel;
		}

		float GetFriction(){
			return friction;
		}
		float GetStaticFriction(){
			return staticFriction;
		}
		virtual float GetMass(){
			return mass;
		}
		float GetRestitution(){
			return restitution;
		}

		float GetCircumference(){
			if(circumferenceNeedsUpdate==true){
				circumference=0.0f;
				for(auto mesh:_meshes){
					circumference+=mesh->GetCircumference();
				}
				circumferenceNeedsUpdate=false;
			}

			return circumference;
		}






		//General Set Methods
		QBody * SetPosition(QVector pos){
			position.x=pos.x;
			position.y=pos.y;
			prevPosition.x=pos.x;
			prevPosition.y=pos.y;
			UpdateMeshTransforms();
			return this;
		}

		QBody * SetPosition(float x, float y){
			position.x=x;
			position.y=y;
			prevPosition.x=x;
			prevPosition.y=y;
			UpdateMeshTransforms();
			return this;
		}
		QBody * SetPreviousPosition(QVector pos){
			prevPosition.x=pos.x;
			prevPosition.y=pos.y;
			return this;
		}
		QBody * SetRotation(float angleRadian){
			rotation=angleRadian;
			prevRotation=angleRadian;
			UpdateMeshTransforms();
			return this;
		}
		QBody *SetPreviousRotation(float angleRadian){
			prevRotation=angleRadian;
			return this;
		}
		QBody *SetLayersBit(int value){
			layersBit=value;
			return this;
		}
		QBody *SetCollidableLayersBit(int value){
			collidableLayersBit=value;
			return this;
		}
		QBody *SetCanSleep(bool value){
			canSleep=value;
			return this;
		}
		QBody * SetMode(QBody::Modes bodyMode){
			mode=bodyMode;
			return this;
		}
		QBody * SetSimulationModel(SimulationModels model)
		{
			simulationModel=model;
			for(auto mesh:_meshes){
				mesh->UpdateCollisionBehavior();
			}
			UpdateMeshTransforms();

			return this;
		}

		QBody *SetFriction(float value){
			friction=value;
			return this;
		}
		QBody *SetStaticFriction(float value){
			staticFriction=value;
			return this;
		}
		QBody *SetMass(float value){
			mass=value;
			interiaNeedsUpdate=true;
			return this;
		}
		QBody *SetRestitution(float value){
			restitution=value;
			return this;
		}


		//Mesh Methods
		QBody * AddMesh(QMesh *mesh);
		QBody * RemoveMeshAt(int index);
		QMesh * GetMeshAt(int index);
		int GetMeshCount();
		vector<QMesh*>  *GetMeshes();







		friend class QMesh;
		friend class QWorld;
		friend class QManifold;
		friend class QParticle;

	protected:
		vector<QMesh*> _meshes=vector<QMesh*>();
		SimulationModels simulationModel=SimulationModels::RIGID_BODY;
		static QVector ComputeFriction(QBody *bodyA, QBody *bodyB, QVector &normal, float penetration, QVector &relativeVelocity);
		static bool CanCollide(QBody *bodyA,QBody *bodyB);


};

#endif // QBODY_H
