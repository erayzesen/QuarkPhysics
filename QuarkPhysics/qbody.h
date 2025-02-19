
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

#ifndef QBODY_H
#define QBODY_H

#include "qvector.h"
#include "qaabb.h"
#include <cmath>
#include <vector>
#include "qmesh.h"
#include <functional>
#include "qmath_utils.h"


class QWorld;
/** @brief QBody objects are the base class for all types of bodies. Any class derived from QBody shares these methods and properties. Additionally, this base type provides flexible and common references for independent operations from qualified and named body types in the entire simulation. Virtual methods are defined for users to create different body types that inherit QBody. For example, the Update() method is used to apply different dynamics for all body types. QRigidBody instances derived from QBody have their own Update() solution, while QSoftBody instances have their own Update() solution. With these and similar virtual methods, unique new body types can be created.
 */
class QBody{
	float inertia=0.0f;
	float circumference=0.0f;
	
public:
	/**
	 * Determines whether the body is dynamic or static. A static body does not react to any force, constraint or collision and does not move.A dynamic body reacts to forces, constraints, collisions, and any other world event.
	 */
	enum Modes{
		DYNAMIC,
		STATIC
	};

	enum BodyTypes{
		RIGID,
		AREA,
		SOFT
	};
	/**
	 * Determines which approach will be used to simulate a body object.
	 */
	enum SimulationModels{
		MASS_SPRING,
		RIGID_BODY
	};

	struct BodyPairHash {
		size_t operator()(const std::pair<QBody*, QBody*>& p) const {
			std::size_t h1 = std::hash<QBody*>{}(p.first);
			std::size_t h2 = std::hash<QBody*>{}(p.second);
			return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
		}
	};

	struct BodyPairEqual {
		bool operator()(const std::pair<QBody*, QBody*>& p1, const std::pair<QBody*, QBody*>& p2) const {
			return (p1.first == p2.first && p1.second == p2.second) ||
				(p1.first == p2.second && p1.second == p2.first);
		}
	};

protected:

	//General Properties

	QWorld *world;
	QVector position=QVector(0,0);
	QVector prevPosition=QVector::Zero();
	float rotation=0.0f;
	float prevRotation=0.0f;
	QAABB aabb;
	Modes mode=QBody::Modes::DYNAMIC;
	bool inertiaNeedsUpdate=true;
	bool circumferenceNeedsUpdate=true;
	bool enableBodySpecificTimeScale=false;
	float bodySpecificTimeScale=1.0f;
	BodyTypes bodyType=BodyTypes::RIGID;
	bool enabled=true;
	float velocityLimit=0.0f;
	bool enableIntegratedVelocities=true;
	bool enableCustomGravity=false;
	QVector customGravity=QVector::Zero();

	//Material Properties;

	float friction=0.2f;
	float staticFriction=0.5f;
	float airFriction=0.01f;
	float mass=1.0f;
	float restitution=0.0f;

	//Collision Features
	int layersBit=1;
	int collidableLayersBit=1;
	bool isKinematic=false;
	bool allowKinematicCollisions=false;

	//Sleeping Features
	bool isSleeping=false;

	int sleepTick=120;
	int fixedVelocityTick=0;
	int fixedAngularTick=0;
	bool canSleep=true;

	

	
	void UpdateAABB();
	void UpdateMeshTransforms();
	/** Updates properties of the body and applies needed physical dynamics. */
	virtual void Update();
	/** Called after all bodies have completed their Update step to perform post-update operations. */
	virtual void PostUpdate(){};
	virtual bool CanGiveCollisionResponseTo(QBody *otherBody);

	public:
		QBody();
		virtual ~QBody();

		//Collision Info
		/** @brief CollisionInfo structure contains collision information of a body object. This information is sent to the relevant event listeners during a collision event.
		 */
		struct CollisionInfo{
			/** The contact position of the collision. */
			QVector position;
			/** The collided body. */
			QBody *body;
			/** The collision normal. */
			QVector normal;
			/** The collision penetration. */
			float penetration;
			CollisionInfo(QVector position,QBody *body,QVector normal,float penetration) : position(position), body(body), normal(normal), penetration(penetration){};
		};

		//Default Events
		/** The event is triggered at the beginning of a physics step, before any collision or constraint operations have been applied to body objects. It is a good time to perform instantaneous operations that will manipulate the presence of a body object in the world. */
		virtual void OnPreStep(){};
		/** The event is triggered after collision and constraint operations are applied in a physics step. It is a good time to perform operations that are not instantaneous but rather intended to be applied in the next physics step on body objects. */
		virtual void OnStep(){};
		/** The event is triggered when a body object collides with another body object during a physics step.
		 * @param CollisionInfo Contains collision informations.
		 * @return If the event method returns true, collision responses are applied. If it returns false, collision responses are not applied.
		 * */
		virtual bool OnCollision(CollisionInfo){ return true;}

		//Custom Event Listeners
		/**  This is the event listener callback function for the OnPreStep event.
		 * @param body The body object that triggers the event. 
		 * */
		std::function<void(QBody *body)> PreStepEventListener;
		/**  This is the event listener callback function for the OnStep event.
		 * @param body The body object that triggers the event. 
		 * */
		std::function<void(QBody *body)> StepEventListener;
		/**  This is the event listener callback function for the OnCollision event.
		 * @param body The body object that triggers the event. 
		 * @param CollisionInfo Contains collision informations.
		 * */
		std::function<bool(QBody *body,CollisionInfo)> CollisionEventListener;




		//General Get Methods

		/** Returns the type of the body. */
		BodyTypes GetBodyType(){
			return bodyType;
		}
		/** Returns the world. */
		QWorld *GetWorld(){
			return world;
		}
		/** Returns the position of the body. */
		QVector GetPosition(){
			return position;
		}
		/** Returns the previous position of the body. */
		QVector GetPreviousPosition(){
			return prevPosition;
		}
		/** Returns the rotation of the body. */
		float GetRotation(){
			return rotation;
		}
		/** Returns the rotation of the body as degree. */
		float GetRotationDegree(){
			return GetRotation()/(M_PI/180);
		}
		/** Returns the previous rotation of the body. */
		float GetPreviousRotation(){
			return prevRotation;
		}
		/** Returns the AABB feature of the body. */
		QAABB GetAABB()const{
			return aabb;
		}

		
		
		/** Returns the total initial area of the body. Initial area means the calculated total area with non-transformed meshes of the body. */
		float GetTotalInitialArea(){
			float res=0.0f;
			for(auto mesh:_meshes){
				res+=mesh->GetInitialArea();
			}
			return res;
		}
		/** Returns the total initial area of the polygons of the body. Initial area means the calculated total area with non-transformed meshes of the body. */
		float GetTotalPolygonsInitialArea(){
			float res=0.0f;
			for(auto mesh:_meshes){
				res+=mesh->GetInitialPolygonsArea();
			}
			return res;
		}
		/** Returns the total area of the body. */
		float GetTotalArea(){
			float res=0.0f;
			for(auto mesh:_meshes){
				res+=mesh->GetArea();
			}
			return res;
		}
		/** Returns the total area of the polygons of the body. */
		float GetTotalPolygonsArea(){
			float res=0.0f;
			for(auto mesh:_meshes){
				res+=mesh->GetPolygonsArea();
			}
			return res;
		}
		/** Returns whether the body is dynamic or static. */
		Modes GetMode(){
			return mode;
		}
		/** Returns the inertia of the body. */
		float GetInertia(){
			if(inertiaNeedsUpdate==true){
				inertia=GetTotalInitialArea()*2.0f*mass;
				inertia=inertia<500.0f ? 500.0f:inertia;
				inertiaNeedsUpdate=false;
			}
			return inertia;
		}
		/** Returns the bit mask that represents the layers in which the body object is present.*/
		int GetLayersBit(){
			return layersBit;
		}
		/** Returns the bit mask that represents the collidable layers in which the body object is present.*/
		int GetCollidableLayersBit(){
			return collidableLayersBit;
		}
		/** Returns whether a body object can collide with other body objects that have the given layers bit as parameters.
		 * @param layersBit A bit value representing the layers to check.  
		 * */
		bool GetOverlapWithCollidableLayersBit(int layersBit){
			if( (layersBit & this->collidableLayersBit)==0 )
				return false;
			return true;
		}
		/**  Checks whether a body object overlaps with a given bit value that represents layers. Returns true if the body object is in at least one of the layers represented by the given bit.
		 * @param layersBit A bit value representing the layers to check.  
		 * */
		bool GetOverlapWithLayersBit(int layersBit){
			if( (layersBit & this->layersBit)==0 ){
				return false;
			}
			return true;
		}
		/** Returns whether the body is sleeping. */
		bool GetIsSleeping(){
			return isSleeping;
		}
		/** Returns whether the body can sleep. */
		bool GetCanSleep(){
			return canSleep;
		}
		/** Returns which approach will be used to simulate a body object. */
		SimulationModels GetSimulationModel(){
			return simulationModel;
		}

		/** Returns the friction value of the body. */
		float GetFriction(){
			return friction;
		}
		/** Returns the static friction value of the body. */
		float GetStaticFriction(){
			return staticFriction;
		}

		/** Returns the air friction value of the body. */
		float GetAirFriction(){
			return airFriction;
		}

		/** Returns the mass value of the body. */
		virtual float GetMass(){
			return mass;
		}
		/** Returns the restitution value of the body. */
		float GetRestitution(){
			return restitution;
		}

		/** Returns the circumference of the body. */
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
		
		/** Returns whether the body spesific time scale is enabled. */
		bool GetBodySpecificTimeScaleEnabled(){
			return enableBodySpecificTimeScale;
		}
		/** Returns the body spesific time scale */

		float GetBodySpesificTimeScale(){
			return bodySpecificTimeScale;
		}

		/** Returns whether the body is enabled.  */
		bool GetEnabled(){
			return enabled;
		}
		/**
		 * Returns the velocity limit of the physics body.  If set to 0, no velocity limit is applied. The default value is 0.
		 */
		float GetVelocityLimit();

		/**
		 * Returns whether the application of gravity and various velocity integrators necessary for the body's movement in the physics world is enabled. It is set to true by default. Typically, it is disabled for specific body objects that require manual control.
		 */
		bool GetIntegratedVelocitiesEnabled();

		/**
		 * Returns whether a specific custom gravity value, defined using the SetCustomGravity() method, will be applied to the body instead of the gravity defined for the physics world.
		 */
		bool GetCustomGravityEnabled();
		
		/*
			Returns the gravity vector specifically defined for the body.
		*/
		QVector GetCustomGravity();

		



		//General Set Methods
		/** Sets the position of the body. 
		 * @param value A position value to set. 
		 * @param withPreviousPosition Determines whether apply the position to the previous position of the body.In this simulation, since velocities are implicit, if the previous position are the same as the newly set position, the positional velocity of the body will also be zeroed.Therefore, if you want to zero out the positional velocity when setting the new position, use this option.
		 * @return A pointer to the body itself.
		 */
		QBody * SetPosition(QVector value, bool withPreviousPosition=true){
			position=value;
			if (withPreviousPosition) {
				prevPosition=position;
			}
			WakeUp();

			UpdateMeshTransforms();
			UpdateAABB();
			return this;
		}

		/** Adds a vector value the position of the body. 
		 * @param value A vector value to add. 
		 * @param withPreviousPosition Determines whether apply the position to the previous position of the body.In this simulation, since velocities are implicit, if the previous position are the same as the newly set position, the positional velocity of the body will also be zeroed.Therefore, if you want to zero out the positional velocity when setting the new position, use this option.
		 * @return A pointer to the body itself.
		 */
		QBody *AddPosition(QVector value, bool withPreviousPosition=true){
			return SetPosition(GetPosition()+value,withPreviousPosition);
		}
		/** Applies a force immediately to the body. The way the force is applied may vary depending on the QBody type, and some QBody types may not respond to the applied forces. You can use the method safely before the physics step (e.g. at the OnPreStep event). If you want to use this method after physics step, it can break the simulation.(Collisions and constraints may not be applied properly.) if you want to apply force at the next physic step safely, use SetForce() and AddForce() methods.  
		 * @param value The force to apply.
		 * @return A pointer to the body itself.
		 */
		virtual  QBody* ApplyForce(QVector value){return this;}; 
		/** Sets the previous position of the body. 
		 * @param value A position value to set. 
		 * @return A pointer to the body itself.
		 */
		QBody * SetPreviousPosition(QVector value){
			prevPosition=value;
			return this; 
		}
		/** Adds a vector the previous position of the body. 
		 * @param value A position value to add. 
		 * @return A pointer to the body itself.
		 */
		QBody *AddPreviousPosition(QVector value){
			return SetPreviousPosition(GetPreviousPosition()+value);
		}
		/** Sets the rotation of the body. 
		 * @param angleRadian A rotation value to set, in radians. 
		 * @param withPreviousRotation Determines whether apply the rotation to the previous rotation of the body.In this simulation, since velocities are implicit, if the previous rotation are the same as the newly set rotation, the angular velocity of the body will also be zeroed.Therefore, if you want to zero out the angular velocity when setting the new rotation, use this option.
		 * @return A pointer to the body itself.
		 */
		QBody * SetRotation(float angleRadian, bool withPreviousRotation=true){
			rotation=angleRadian;
			if(withPreviousRotation)
				prevRotation=angleRadian;
			WakeUp();
			UpdateMeshTransforms();
			return this;
		}
		/** Sets the rotation of the body with specified angle in degrees. 
		 * @param degree A rotation value to set, in degrees. 
		 * @param withPreviousRotation Determines whether apply the rotation to the previous rotation of the body.In this simulation, since velocities are implicit, if the previous rotation are the same as the newly set rotation, the angular velocity of the body will also be zeroed.Therefore, if you want to zero out the angular velocity when setting the new rotation, use this option.
		 * @return A pointer to the body itself.
		 */
		QBody * SetRotationDegree(float degree, bool withPreviousRotation=true){
			return SetRotation( degree*(M_PI/180.0f),withPreviousRotation );
		}
		
		/** Adds a value to the rotation of the body. 
		 * @param angleRadian A value to add, in radians. 
		 * @param withPreviousRotation Determines whether apply the rotation to the previous rotation of the body.In this simulation, since velocities are implicit, if the previous rotation are the same as the newly set rotation, the angular velocity of the body will also be zeroed.Therefore, if you want to zero out the angular velocity when setting the new rotation, use this option.
		 * @return A pointer to the body itself.
		 */
		QBody *AddRotation(float angleRadian, bool withPreviousRotation=true){
			return SetRotation(GetRotation()+angleRadian,withPreviousRotation);
		}
		/** Sets the previous rotation of the body. 
		 * @param angleRadian A rotation value to set, in radians. 
		 * @return A pointer to the body itself.
		 */
		QBody *SetPreviousRotation(float angleRadian){
			prevRotation=angleRadian;
			return this;
		}
		/** Adds a value to the previous rotation of the body. 
		 * @param angleRadian A value to add, in radians. 
		 * @return A pointer to the body itself.
		 */
		QBody *AddPreviousRotation(float angleRadian){
			return SetPreviousRotation(GetPreviousRotation()+angleRadian);
		}



		
		/** Sets the bit mask that represents the layers in which the body object is present.
		 * @param value A bit mask value to set. 
		 * @return A pointer to the body itself.
		 */
		QBody *SetLayersBit(int value){
			layersBit=value;
			return this;
		}
		/** Sets the bit mask that represents the collidable layers in which the body object is present.A body object can collide with other body objects present in the layers defined by the user. 
		 * @param value A bit mask value to set. 
		 * @return A pointer to the body itself.
		 */
		QBody *SetCollidableLayersBit(int value){
			collidableLayersBit=value;
			return this;
		}
		/** Sets whether the body can sleep in the active sleep mode.
		 * @param value True or false. 
		 * @return A pointer to the body itself.
		 */
		QBody *SetCanSleep(bool value){
			canSleep=value;
			return this;
		}
		/** Sets whether the body is dynamic or static. 
		 * @param bodyMode A mode to set. 
		 * @return A pointer to the body itself.
		 */
		QBody * SetMode(QBody::Modes bodyMode){
			mode=bodyMode;
			return this;
		}
		/** Sets which approach will be used to simulate a body object.
		 * @param bodyMode A simulation model to set. 
		 * @return A pointer to the body itself.
		 */
		QBody * SetSimulationModel(SimulationModels model)
		{
			simulationModel=model;
			for(auto mesh:_meshes){
				mesh->UpdateCollisionBehavior();
			}
			UpdateMeshTransforms();

			return this;
		}
		/** Sets the friction value of the body. 
		 * @param value A value to set.
		 * @return A pointer to the body itself.
		 */
		QBody *SetFriction(float value){
			friction=value;
			return this;
		}
		/** Sets the static friction value of the body. 
		 * @param value A value to set.
		 * @return A pointer to the body itself.
		 */
		QBody *SetStaticFriction(float value){
			staticFriction=value;
			return this;
		}

		/** Sets the air friction value of the body. 
		 * @param value A value to set.
		 * @return A pointer to the body itself.
		 */
		QBody *SetAirFriction(float value){
			airFriction=value;
			return this;
		}

		/** Sets the mass value of the body. 
		 * @param value A value to set.
		 * @return A pointer to the body itself.
		 */
		QBody *SetMass(float value){
			mass=value;
			inertiaNeedsUpdate=true;
			return this;
		}
		/** Sets the restitution value of the body. 
		 * @param value A value to set.
		 * @return A pointer to the body itself.
		 */
		QBody *SetRestitution(float value){
			restitution=value;
			return this;
		}

		/** Sets whether the body-specific time scale is enabled. The world has a time scale, but this allows you to assign a specific time scale to the body object as an exception. 
		 * @param value A value to set.
		 * @return A pointer to the body itself.
		 */
		QBody *SetBodySpecificTimeScaleEnabled(bool value){
			enableBodySpecificTimeScale=value;
			return this;
		}

		/** Sets the time scale specific to the body. The world has a time scale, but this allows you to assign a specific time scale to the body object as an exception.  
		 * @param value A value to set.
		 * @return A pointer to the body itself.
		 */
		QBody *SetBodySpecificTimeScale(float value){
			bodySpecificTimeScale=value;
			return this;
		}
		/** Sets whether the body is enabled.  
		 * @param value A value to set
		 * @return A pointer to the body itself.
		 */
		QBody *SetEnabled(bool value){
			enabled=value;
			return this;
		}

		/**
		 * Sets whether the application of gravity and various velocity integrators necessary for the body's movement in the physics world is enabled. It is set to true by default. Typically, it is disabled for specific body objects that require manual control.
		 * @param value A value to set
		 * @return A pointer to the body itself.
		 */
		QBody *SetIntegratedVelocitiesEnabled(bool value);

		/**
		 * Sets whether a specific custom gravity value, defined using the SetCustomGravity() method, will be applied to the body instead of the gravity defined for the physics world.
		 * @param value A value to set
		 * @return A pointer to the body itself.
		 */
		QBody *SetCustomGravityEnabled(bool value);

		/**
		 * Sets whether a specific custom gravity value, defined using the SetCustomGravity() method, will be applied to the body instead of the gravity defined for the physics world.
		 * @param value A value to set
		 * @return A pointer to the body itself.
		 */
		QBody *SetCustomGravity(QVector value);


		//Mesh Methods
		/** Adds a mesh to the body.
		 * @param mesh A pointer of the mesh to add.
		 * @return A pointer to the body itself.
		 */
		QBody * AddMesh(QMesh *mesh);
		/** Removes a mesh from the body.
		 * @param index The index of the mesh to remove.
		 * @return A pointer to the body itself.
		 */
		QBody * RemoveMeshAt(int index);
		/** Returns the mesh of the body with specified index.
		 * @param index The index of the mesh.
		 */
		QMesh * GetMeshAt(int index);
		/** Returns the count of the meshes of to the body. */
		int GetMeshCount();
		/** Returns the collection of the meshes of the body. */
		vector<QMesh*>  *GetMeshes();

		/** Adds meshes with a json based *.qmesh file.
		 * @param filePath A file path.
		 * @return A pointer to the body itself.
		 */
		QBody * AddMeshesFromFile(string filePath);


		//Methods About the Sleeping Feature
		/**
		 * Wakes up a body that is in a sleeping state.
		 * @return A pointer to the body itself.
		 */
		QBody* WakeUp() {
			isSleeping = false;
			return this;
		}

		/**
		 * Limits the velocity of the physics body. If set to 0, no velocity limit is applied. The default value is 0.
		 * @return A pointer to the body itself.
		 */
		QBody *SetVelocityLimit(float value);

		/**
		 * By default, objects included in the physics engine are deleted by the destructors of the objects they belong to. When this flag is enabled, it indicates that this object should never be deleted by this engine. It is disabled by default, and it is recommended to keep it disabled. However, it can be used if needed for advanced purposes and integrations.
		 */
		bool manualDeletion=false;


		friend class QMesh;
		friend class QWorld;
		friend class QManifold;
		friend class QParticle;
		friend class QJoint;
		friend class QBroadPhase;
		friend class QAreaBody;

	protected:
		vector<QMesh*> _meshes=vector<QMesh*>();
		SimulationModels simulationModel=SimulationModels::RIGID_BODY;
		static QVector ComputeFriction(QBody *bodyA, QBody *bodyB, QVector &normal, float penetration, QVector &relativeVelocity);
		static bool CanCollide(QBody *bodyA,QBody *bodyB,bool checkBodiesAreEnabled=true);
		//For Gravity-Free Feature of QArea Bodies  
		bool ignoreGravity=false;



};

#endif // QBODY_H
