
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

#ifndef QPARTICLE_H
#define QPARTICLE_H
#include "qvector.h"
#include <vector>

class QMesh;
/** @brief QParticle objects form the network structures of QMesh objects defined for all body object types. They are the smallest building blocks of physics simulation and are manipulated differently in different body object types. For example, in QRigidBody objects, particles are collectively forced into positions obtained through various calculations based on the current body properties. However, in soft body objects, simulation particles are individually manipulated and can move freely, determining the next steps of the simulation through their individual movements. QMesh objects offer a number of methods to manage particles. For more information on restrictions between particles in soft body objects, see the QSpring object.
 */
class QParticle
{
	float r=0.5f;

	QVector globalPosition;
	QVector prevGlobalPosition;

	QVector position;

	float mass=1.0f;

	QMesh *ownerMesh=nullptr;

	bool isInternal=false;

	QVector force=QVector::Zero();

	std::vector<QVector> accumulatedForces;
public:
	QParticle();
	QParticle(float posX,float posY,float radius=0.5f);
	QParticle(QVector pos,float radius=0.5f);



	//Get Methods
	/** Returns the global position of the particle. */
	QVector GetGlobalPosition(){
		return globalPosition;
	}
	/** Returns the previous global position of the particle. */
	QVector GetPreviousGlobalPosition(){
		return prevGlobalPosition;
	}
	/** Returns the local position of the particle. */
	QVector GetPosition(){
		return position;
	}
	/** Returns the mass of the particle. */
	float GetMass(){
		return mass;
	}
	/** Returns owner mesh of the particle. 
	 * The Owner mesh is the mesh in which the particle is appointed. 
	 */
	QMesh * GetOwnerMesh(){
		return ownerMesh;
	}
	/** Returns the radius of the particle. */
	float GetRadius(){
		return r;
	}
	/** Returns whether the particle is internal. Internal particle definition is used not for the particles that define the collision boundaries of a mesh, but for the grid particles inside these boundaries. This feature is important for simulation types that require different internal particle simulation, such as volume preserved soft bodies.*/
	bool GetIsInternal(){
		return isInternal;
	}
	/** Returns the current force value of the particle. */
	QVector GetForce(){
		return force;
	}

	//Set Methods
	/** Sets the local position of the particle. 
	 * @param value A position value to set. 
	 * @return A pointer to the particle itself.
	 */
	QParticle *SetGlobalPosition(QVector value);
	/** Sets the global position of the particle. 
	 * @param value A position value to set. 
	 * @return A pointer to the particle itself.
	 */
	QParticle *AddGlobalPosition(QVector value);
	/** Sets the previous global position of the particle. 
	 * @param value A position value to set. 
	 * @return A pointer to the particle itself.
	 */
	QParticle *SetPreviousGlobalPosition(QVector value);
	/** Adds a value to the previous global position of the particle. 
	 * @param value A value to add. 
	 * @return A pointer to the particle itself.
	 */
	QParticle *AddPreviousGlobalPosition(QVector value);
	QParticle *SetPosition(QVector value);
	/** Adds a value to the local position of the particle. 
	 * @param value A value to add. 
	 * @return A pointer to the particle itself.
	 */
	QParticle *AddPosition(QVector value);

	/** Sets the mass of the particle. 
	 * @param value A value to set. 
	 * @return A pointer to the particle itself.
	 */
	QParticle *SetMass(float value);
	/** Sets the owner mesh of the particle. Usually the owner mesh is defining when a particle added to a mesh.  
	 * @param mesh A pointer of the mesh to set. 
	 * @return A pointer to the particle itself.
	 */
	QParticle *SetOwnerMesh(QMesh *mesh);
	/** Sets the radius of the particle. 
	 * @param radius A value to set. 
	 * @return A pointer to the particle itself.
	 */
	QParticle *SetRadius(float radius);
	/** Sets whether the particle is internal.Internal particle definition is used not for the particles that define the collision boundaries of a mesh, but for the grid particles inside these boundaries. This feature is important for simulation types that require different internal particle simulation, such as volume preserved soft bodies.
	 * @param value True or false. 
	 * @return A pointer to the particle itself.
	 */
	QParticle *SetIsInternal(bool value);

	//
	/** Applies a force immediately to the particle. You can use the method safely before the physics step (e.g. at the OnPreStep event of QBody objects). If you want to use this method after physics step, it can break the simulation.(Collisions and constraints may not be applied properly.) if you want to apply force at the next physic step safely, use SetForce() and AddForce() methods.  
	 * @param value A force value to apply. 
	 * */
	QParticle *ApplyForce(QVector value);
	/** Sets the force value of the particle. Set forces determine the force to be applied to a particle object at the next physics step from the current step. 
	 * @param value A value to set. 
	 * @return A pointer to the particle itself.
	 */
	QParticle *SetForce(QVector value);
	/** Adds a value to the force of the particle.Set forces determine the force to be applied to a particle object at the next physics step from the current step.  
	 * @param value A value to add. 
	 * @return A pointer to the particle itself.
	 */
	QParticle *AddForce(QVector value);


	//Accumulated Forces
	/** Adds a new force to the accumulated forces. The purpose of the accumulated forces is to apply their arithmetic average using ApplyAccumulatedForce(). Before performing operations with the accumulated forces, they must be cleared using ClearAccumulatedForce().
	 * @param value A value to add. 
	 * @return A pointer to the particle itself.
	 */
	QParticle *AddAccumulatedForce(QVector value);
	/** Clears the accumulated forces. It should be called before working with accumulated forces. Additionally, the accumulated forces are automatically cleared after the ApplyAccumulatedForce() method is called. */
	QParticle *ClearAccumulatedForces();
	/** Calculates the arithmetic average of the accumulated forces based on the number of forces, applies it to the particle, and then clears all forces. */
	QParticle *ApplyAccumulatedForces();


	//Static Methods
	/** Applies a specified force to a segment created by two particles at a specific position. 
	 * @param pA A particle in the world.
	 * @param pB Another particle in the world.
	 * @param force A force value to apply.
	 * @param fromPosition The position of the force. 
	 */
	static void ApplyForceToParticleSegment(QParticle *pA,QParticle *pB,QVector force,QVector fromPosition);
	
};

#endif // QPARTICLE_H
