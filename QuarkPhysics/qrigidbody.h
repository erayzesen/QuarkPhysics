
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

#ifndef QRIGIDBODY_H
#define QRIGIDBODY_H
#include "qbody.h"
#include "qvector.h"

/** @brief QRigidBody is a type of body that is simulated with the dynamics of Rigid body. A rigid body is a type of object in physics simulations that models non-deformable, solid objects. Rigid bodies have properties such as momentum, center of mass, inertia, and mass, which affect their simulation. These properties are used to compute the motion and collisions of rigid bodies in a physics engine.

 *Rigid bodies are different from soft bodies and other particle-based physics objects, in that they are typically manipulated by adjusting their position and orientation directly, rather than by manipulating individual particles. The transformations applied to a rigid body are then used to update the positions and orientations of any associated meshes or graphics objects.

 *Rigid bodies are used to simulate a wide range of objects in physics simulations, such as rectangles, circles and complex shapes and are a fundamental part of most physics engines.
 */
class QRigidBody : public QBody
{
	bool fixedRotation=false;
protected:
	QVector force=QVector::Zero();
	float angularForce=0.0f;
public:
	QRigidBody();

	//Get Methods
	/** Returns whether the fixed rotation option is enabled. */
	bool GetFixedRotationEnabled(){
		return fixedRotation;
	}
	/** Returns whether kinematic option is enabled. */
	bool GetKinematicEnabled(){
		return isKinematic;
	}
	/** Sets whether collisions against other kinematic bodies are enabled. */
	bool GetKinematicCollisionsEnabled(){
		return allowKinematicCollisions;
	}

	/** Returns the current force value of the body. */
	QVector GetForce(){
		return force;
	}
	/** Returns the current angular force value of the body. */
	float GetAngularForce(){
		return angularForce;
	}

	//Set Methods
	/** Sets whether the fixed rotation option is enabled. If set to true, the rotation value of the body never affected with the physics simulation. */
	QRigidBody* SetFixedRotationEnabled(bool value){
		fixedRotation=value;
		return this;
	}
	/** Sets whether the kinematic option is enabled. Physical interactions such as gravity and acceleration are not applied to Kinematic Body, and are not affected by collisions with dynamic objects. The values of these objects such as position and rotation are under the control of the user.   */
	QRigidBody* SetKinematicEnabled(bool value){
		isKinematic=value;
		return this;
	}
	/** Sets whether collisions against other kinematic bodies are enabled. By default, a kinematic body is not affected by collisions with other kinematic bodies. However, if set to true, a kinematic body can react to collisions with other kinematic bodies.
	 */
	QRigidBody * SetKinematicCollisionsEnabled(bool value){
		allowKinematicCollisions=value;
		return this;
	}

	/** Sets the position of the body and collides with other bodies. If you need to set the position after the physics step and you don't want to break the simulation, you can use this method. If you change the position of the body on the before the physics step ( e.g. at the OnPreStep event ), you don't need to use it.   
	 * @param value A position to set.
	 * @param withPreviousPosition Determines whether apply the position to the previous position of the body.In this simulation, since velocities are implicit, if the previous positions are the same as the newly set position, the positional velocity of the body will also be zeroed.Therefore, if you want to zero out the positional velocity when setting the new position, use this option.
	 */
	QRigidBody * SetPositionAndCollide(QVector value,bool withPreviousPosition=true);


	//#Rigidbody Methods
	/** Applies a force immediately to the body. You can use the method safely before the physics step (e.g. at the OnPreStep event). If you want to use this method after physics step, it can break the simulation.(Collisions and constraints may not be applied properly.) if you want to apply force at the next physic step safely, use SetForce() and AddForce() methods.  
	 * @param force The force to apply.
	 * @param r The relative position to apply force.
	 * @param updateMeshTransforms Determines whether to update the transforms of the meshes. It is recommended to set this option to true in general use. If set to false, it changes the values such as position and rotation of the applied force, but does not update the corresponding mesh properties. This option is provided for optimization advantage for objects that are subjected to a lot of processing and where updating mesh transform until the end of the process is not important. In such cases, you can call the UpdateMeshTransforms() method externally after the process is completed.
	 */
	QRigidBody* ApplyForce(QVector force,QVector r,bool updateMeshTransforms=true);
	/** Applies a impulse immediately to the body. Since velocity values are implicit in this physical simulation, impulses are applied to previousPosition and previousRotation properties of the body. 
	 * @param impulse The impulse to apply.
	 * @param r The relative position to apply impulse.
	 */
	QRigidBody* ApplyImpulse(QVector impulse,QVector r);
	/** Sets the force value of the body. Set forces determine the force to be applied to a body object at the next physics step from the current step. 
	 * @param value A value to set. 
	 * @return A pointer to the body itself.
	 */
	QRigidBody *SetForce(QVector value);
	/** Adds a vector to the force value of the body. Set forces determine the force to be applied to a body object at the next physics step from the current step. 
	 * @param value A value to add. 
	 * @return A pointer to the body itself.
	 */
	QRigidBody *AddForce(QVector value);

	/** Sets the angular force of the body. 
	 * @param value A value to set. 
	 * @return A pointer to the body itself.
	 */
	QRigidBody *SetAngularForce(float value);
	/** Adds a value to the angular force of the body. 
	 * @param value A value to add. 
	 * @return A pointer to the body itself.
	 */
	QRigidBody *AddAngularForce(float value);


	/** Updates properties of the rigid body and applies needed physical dynamics. */
	void Update();


};

#endif // QRIGIDBODY_H
