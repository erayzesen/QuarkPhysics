
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

#ifndef QSOFTBODY_H
#define QSOFTBODY_H
#include "qbody.h"
#include <utility>
/** @brief QSoftBody is a body type that defines deformable, soft objects in the physics world. Mass-spring model is used for simulation dynamics in soft bodies. In the mass-spring model, there are particles with mass that can move individually and interact with the physics world, and these particles can be connected to each other with spring constraints. Additionally, with some user-configurable options specific to the simulation, particles can be subjected to constraints obtained from some calculations. For example, you can add a constraint that ensures particles remain faithful to their initially defined local positions using the "shape matching" option. You can apply a constraint that gives the feeling that the polygon are filled with gas and maintains their area using the "area preserving" option. You can use options that allow particles to collide with each other with a specific radius, and create objects called PBD (Position Based Dynamics). QSoftBody objects inherently require a more flexible configuration than other body types and contain many options.
 */
class QSoftBody : public QBody
{
	//#Softbody Properties
	float rigidity=1.0f;
	float enableAreaPreserving=false;
	float areaPreservingRate=0.8f;
	float areaPreservingRigidity=1.0f;
	float targetPreservationArea=0.0f;

	float particleSpesificMass=1.0f;
	bool enableParticleSpesificMass=false;


	float circumference=0.0f;
	bool enableAreaStability=false;
	bool enablePassivationOfInternalSprings=false;


	bool enableSelfCollisions=false;
	float selfCollisionParticleRadius=0.0f;

	bool enableShapeMatching=false;
	float shapeMatchingRate=0.4f;
	bool ApplyShapeMatchingInternals=false;
	bool enableShapeMatchingFixedTransform=false;
	QVector shapeMatchingFixedPosition=QVector::Zero();
	float shapeMatchingFixedRotation=0.0f;


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

	bool IsPolygonCW(vector<QParticle*> polygon);


public:
	QSoftBody();



	//Properties Set Methods (it returns this)
	/** Sets the rigidity of the body. It determines the rigidity of the spring joints of the body.  
	 * @param value A value to set. 
	 *  @return A pointer to the body itself.
	 */
	QSoftBody * SetRigidity(float value){
		rigidity=value;
		return this;
	};
	/** Sets the rate to apply area preserving to the body if the area preserving is enabled. Determines the rate of the target area to apply preserve constraints.  
	 * @param value A value to set, it must be a value between 0.0 and 1.0. 
	 *  @return A pointer to the body itself.
	 */
	QSoftBody * SetAreaPreservingRate(float value){
		areaPreservingRate=value;
		return this;
	}
	/** Sets the rigidity to apply area preserving to the body if the area preserving is enabled. Determines the hardness of the restrictions to be applied to particles for the application of the target area.
	 * @param value A value to set. 
	 *  @return A pointer to the body itself.
	 */
	QSoftBody * SetAreaPreservingRigidity(float value){
		areaPreservingRigidity=value;
		return this;
	}
	/** Sets whether the area preserving option is enabled of the body. If the area preserving option is enabled, the total area of the polygon are calculated at each physics step, and forces are applied to the particles that define the boundaries of the polygon to achieve a total area that can be determined by the user. If the user does not specify, when this option is enabled, the target area is calculated based on the original positions of the polygon particles, in other words, the total area of undeformed polygons are set as the target area.
	 * @param value A value to set. 
	 * @return A pointer to the body itself.
	 */
	QSoftBody * SetAreaPreservingEnabled(bool value){
		enableAreaPreserving=value;
		if(value==true){
			targetPreservationArea=GetTotalPolygonsInitialArea();
		}
		return this;
	}

	/** Sets the total target area to apply area preserving if the area preserving is enabled. 
	 * @param value A value to set. 
	 *  @return A pointer to the body itself.
	 */
	QSoftBody *SetTargetPreservationArea(float value){
		targetPreservationArea=value;
		return this;
	}
	/** Sets whether self particle collisions are enabled for the body. If set to true, all mesh parts within the body will collide with each other.
	 * @param value A value to set. 
	 *  @return A pointer to the body itself.
	 */
	QSoftBody *SetSelfCollisionsEnabled(bool value){
		enableSelfCollisions=value;
		return this;
	}

	/** Sets a specified particle radius value for self particle collisions. If set to 0, particles will collide with their radius. Default value is 0.  
	 * @param value A value to set. 
	 *  @return A pointer to the body itself.
	 */
	QSoftBody *SetSelfCollisionsSpecifiedRadius(float value){
		selfCollisionParticleRadius=value;
		return this;
	}

	/** Sets whether to passivate the internal spring connections of the soft body. If this option is enabled, the internal springs are more passive in the simulation, which can be useful for soft bodies where the internal springs and particle connections only provide UV and other data based on the movement of the soft body.
	 * @param value A value to set. 
	 *  @return A pointer to the body itself.
	 */
	QSoftBody *SetPassivationOfInternalSpringsEnabled(bool value){
		enablePassivationOfInternalSprings=value;
		return this;
	}
	/** Sets whether the shape matcing option is enabled for the body. If set to true, during the simulation process, all particles are forced to stay true to their undeformed positions.
	 * @param value A value to set.
	 * @param withoutInternals Applies method without internal particles.
	 *  @return A pointer to the body itself.
	 */
	QSoftBody *SetShapeMatchingEnabled(bool value, bool withoutInternals=false){
		enableShapeMatching=value;
		ApplyShapeMatchingInternals=!withoutInternals;
		return this;
	}

	/** Sets the rate value to apply the shape matching to the body.   
	 * @param value A value to set, it must be a value between 0.0 and 1.0. 
	 *  @return A pointer to the body itself.
	 */
	QSoftBody *SetShapeMatchingRate(float value){
		shapeMatchingRate=value;
		return this;
	}

	/** Sets whether you will be able to determine the position and rotation of the target shape when the shape matching feature is active. If set to true, you will be able to adjust the position and rotation values of the target shape yourself. 
	 * @param value A value to set. 
	 *  @return A pointer to the body itself.
	 */
	QSoftBody *SetShapeMatchingFixedTransformEnabled(bool value){
		enableShapeMatchingFixedTransform=value;
		shapeMatchingFixedPosition=GetPosition();
		shapeMatchingFixedRotation=GetRotation();
		return this;
	}

	/** Sets the position of the target shape during shape matching if the fixed transform feature is active. 
	 * @param value A value to set. 
	 *  @return A pointer to the body itself.
	 */
	QSoftBody *SetShapeMatchingFixedPosition(QVector value){
		shapeMatchingFixedPosition=value;
		return this;
	}

	/** Sets the rotation value of the target shape during shape matching if the fixed transform feature is active. 
	 * @param value A value to set. 
	 *  @return A pointer to the body itself.
	 */
	QSoftBody *SetShapeMatchingFixedRotation(float value){
		shapeMatchingFixedRotation=value;
		return this;
	}
	/** Sets the particle-specific mass value of the body. This value is used if the particle-specific mass option is enabled.
	 * @param value A value to set.
	 *  @return A pointer to the body itself.
	 */
	QSoftBody *SetParticleSpesificMass(float value){
		particleSpesificMass=value;
		return this;
	}

	/** Sets whether the particle-spesific mass option is enabled for the body. If this option is set to true, particles will continue to move with the mass defined for the body, but a specific mass value provided by you will be used for collision and constraint calculations.  In most case, you don't want the mass value defined for the entire soft body object to be used for individual particle collision and constraint responses. Instead, you can define a specific particle mass value by dividing the mass value of the body by the number of particles, for example. 
	 * @param value A value to set. 
	 *  @return A pointer to the body itself.
	 */
	QSoftBody *SetParticleSpesificMassEnabled(bool value){
		enableParticleSpesificMass=value;
		return this;
	}

	//Properties Get Methods
	/** Returns mass value of the body. */
	float GetMass(){
		if(enableParticleSpesificMass)
			return particleSpesificMass;
		else
			return mass;
	}
	/** Returns the rigidity of the body. It determines the rigidity of the spring joints of the body. */ 
	float  GetRigidity(){
		return rigidity;
	};
	/** Returns the rate to apply area preserving to the body if the area preserving is enabled. Determines the rate of the target area to apply preserve constraints. */ 
	float  GetAreaPreservingRate(){
		return areaPreservingRate;
	};
	/** Returns the rigidity to apply area preserving to the body if the area preserving is enabled. Determines the hardness of the restrictions to be applied to particles for the application of the target area. */
	float  GetAreaPreservingRigidity(){
		return areaPreservingRigidity;
	};
	/** Returns whether the area preserving option is enabled of the body. If the area preserving option is enabled, the total area of  the polygon are calculated at each physics step, and forces are applied to the particles that define the boundaries of the polygon to achieve a total area that can be determined by the user. If the user does not specify, when this option is enabled, the target area is calculated based on the original positions of the polygon particles, in other words, the total area of undeformed polygons are set as the target area. */
	bool GetAreaPreservingEnabled(){
		return enableAreaPreserving;
	}
	/** Returns the total target area to apply area preserving if the area preserving is enabled.*/ 
	float GetTargetPreservationArea(){
		return targetPreservationArea;
	}
	/** Gets whether self collisions are enabled for the body. If set to true, all mesh parts within the body will collide with each other. */
	bool GetSelfCollisionsEnabled(){
		return enableSelfCollisions;
	}
	/**  Returns the specified particle radius value for self particle collisions. If the value is 0.0, particles will collide with their radius. Default value is 0.0.  */
	float GetSelfCollisionsSpecifiedRadius(){
		return selfCollisionParticleRadius;
	}
	/** Returns whether to passivate the internal spring connections of the soft body. If this option is enabled, the internal springs are more passive in the simulation, which can be useful for soft bodies where the internal springs and particle connections only provide UV and other data based on the movement of the soft body. */
	bool GetPassivationOfInternalSpringsEnabled(){
		return enablePassivationOfInternalSprings;
	}
	/** Returns whether the shape matcing option is enabled for the body. If set to true, during the simulation process, all particles are forced to stay true to their undeformed positions.*/
	bool GetShapeMatchingEnabled(){
		return enableShapeMatching;
	}
	

	/** Returns the rate value to apply the shape matching to the body.  */ 
	float GetShapeMatchingRate(){
		return shapeMatchingRate;
	}

	/** Returns whether you will be able to determine the position and rotation of the target shape when the shape matching feature is active. If set to true, you will be able to adjust the position and rotation values of the target shape yourself.  */
	bool GetShapeMatchingFixedTransformEnabled(){
		return enableShapeMatchingFixedTransform;
	}

	/** Sets the position of the target shape during shape matching if the fixed transform feature is active. */
	QVector GetShapeMatchingFixedPosition(){
		return shapeMatchingFixedPosition;
	}

	/** Sets the rotation of the target shape during shape matching if the fixed transform feature is active.  */
	float GetShapeMatchingFixedRotation(){
		return shapeMatchingFixedRotation;
	}

	/** Returns the particle-specific mass value of the body. This value is used if the particle-specific mass option is enabled. */
	float GetParticleSpesificMass(){
		return particleSpesificMass;
	}
	/** Sets whether the particle-spesific mass option is enabled for the body. If this option is set to true, particles will continue to move with the mass defined for the body, but a specific mass value provided by you will be used for collision and constraint calculations.  In most case, you don't want the mass value defined for the entire soft body object to be used for individual particle collision and constraint responses. Instead, you can define a specific particle mass value by dividing the mass value of the body by the number of particles, for example.*/ 
	bool GetParticleSpesificMassEnabled(){
		return enableParticleSpesificMass;
	}

	/** Applies a force immediately to the all particles of the soft body. You can use the method safely before the physics step (e.g. at the OnPreStep event). If you want to use this method after physics step, it can break the simulation.(Collisions and constraints may not be applied properly.) if you want to apply force at the next physic step safely, use SetForce() and AddForce() methods.  
	 * @param force The force to apply.
	 */
	virtual QSoftBody* ApplyForce(QVector force) override;


	//
	/** Updates properties of the soft body and applies needed physical dynamics. */
	virtual void Update();
	/** Called after all bodies have completed their Update step to perform post-update operations. */
	virtual void PostUpdate();
	/** Applies the preserve area operation to the body. */ 
	void PreserveAreas();
	/** Applies the shape matching operation to the body. */ 
	void ApplyShapeMatching();

	

};

#endif // QSOFTBODY_H
