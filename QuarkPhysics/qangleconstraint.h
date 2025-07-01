
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

#ifndef QANGLECONSTRAINT_H
#define QANGLECONSTRAINT_H
#include "qparticle.h"

class QMesh;
/**
 *@brief You can apply angle constraints between 3 particles using the QAngleConstraint. The physics engine uses QAngleConstraint to impose angle constraints between particles in objects already simulated using mass-spring models (e.g. QSoftBody objects). However, if the user wants, they can apply specific angle constraints between any 3 particles using QAngleConstraint. QWorld also provides methods to manage QAngleConstraint objects.
 *\warning QAngleConstraint will not work correctly on objects that are not simulated with particle dynamics, such as rigid bodies whose positions are fixed by transformations. QAngleConstraint relies on the ability to change the positions of particles with freedom of movement to apply constraints correctly.
 */
class QAngleConstraint
{
	float rigidity=0.5f;
	QParticle *pA;
	QParticle *pB;
	QParticle *pC;
	float minAngle=0.0;
	float maxAngle=0.0;
	float currentAngle=0.0f;
	float prevAngle=0.0f;
	bool enabled=true;
	bool beginToSaveAngles=true;
	

public:
	/**
	 * Creates an angle constraint between three particles. But auto calculates min-max angle with the local positions of the particles.   
	 * @param particleA A particle in the world.
	 * @param particleB Another particle in the world.
	 * @param particleC Another particle in the world.
	 */
	QAngleConstraint(QParticle *particleA,QParticle *particleB,QParticle *particleC,float angleRange=0.1f );
	/**
	 * Creates an angle constraint between three particles.
	 * @param particleA A particle in the world.
	 * @param particleB Another particle in the world.
	 * @param particleC Another particle in the world.
	 * @param minimumAngle The min-angle between three particles.
	 * @param maximumAngle The max-angle between the three particles.
	 */
	QAngleConstraint(QParticle *particleA,QParticle *particleB,QParticle *particleC,float minimumAngle,float maximumAngle);



	/**
	 * Applies angle constraints and updates particle positions.
	 * @param specifiedRigidity This is the specific rigidity value that will be used instead of the current rigidity value of the constraint. By default, it is set to -1.0, which means it is disabled. It must be a value between 0.0 and 1.0. Otherwise, the original rigidity value defined for the constraint will be applied. 
	 * @param addToAccumulatedForces This property allows forces to be accumulated on particles for advanced purposes and then applies the average of these forces to the particle. If set to true, the necessary force for the constraint is not applied directly but is instead added to the particle's accumulated force collection. These properties are defined in the QParticle class and should only be used if you truly understand what you are doing."
	 */
	virtual void Update(float specifiedRigidity=-1.0f,bool addToAccumulatedForces=false);

	//Get Methods
	/** Returns particleA of the angle constraint */
	QParticle *GetParticleA(){
		return pA;
	}
	/** Returns particleB of the angle constraint */
	QParticle *GetParticleB(){
		return pB;
	}
	/** Returns particleC of the angle constraint */
	QParticle *GetParticleC(){
		return pC;
	}
	/** Returns the min-angle of the constraint. */
	float GetMinAngle(){
		return minAngle;
	}
	/** Returns the max-angle of the constraint */
	float GetMaxAngle(){
		return maxAngle;
	}
	/** Returns the rigidity of the angle constraint */
	float GetRigidity(){
		return rigidity;
	}
	/** Returns whether the angle constraint is enabled. */
	bool GetEnabled(){
		return enabled;
	}

	float GetCurrentAngle(){
		return currentAngle;
	}

	//Set Methods
	/** Sets particleA of the angle constraint.
	 * @param particle A pointer to the particle.
	 * @return A pointer to the spring itself.
	 */
	QAngleConstraint *SetParticleA(QParticle *particle){
		pA=particle;
		return this;
	}
	/** Sets particleB of the angle constraint.
	 * @param particle A pointer to the particle.
	 * @return A pointer to the spring itself.
	 */
	QAngleConstraint *SetParticleB(QParticle *particle){
		pB=particle;
		return this;
	}

	/** Sets particleC of the angle constraint.
	 * @param particle A pointer to the particle.
	 * @return A pointer to the spring itself.
	 */
	QAngleConstraint *SetParticleC(QParticle *particle){
		pC=particle;
		return this;
	}
	/** Sets the min-angle of the constraint.
	 * @param value The angle to set.
	 * @return A pointer to the spring itself.
	 */
	QAngleConstraint *SetMinAngle(float value){
		minAngle=value;
		return this;
	}

	/** Sets the max-angle of the constraint.
	 * @param value The angle to set.
	 * @return A pointer to the spring itself.
	 */
	QAngleConstraint *SetMaxAngle(float value){
		maxAngle=value;
		return this;
	}
	
	/** Sets the rigidity of the constraint.
	 * @param rigidity The rigidity value to set.It must be a value between 0.0 and 1.0.
	 * @return A pointer to the spring itself.
	 */
	QAngleConstraint *SetRigidity(float rigidity){
		this->rigidity=rigidity;
		return this;
	}
	/** Sets whether the angle constraint is enabled. 
	 * @param value True or false.
	 * @return A pointer to the spring itself.
	 */
	QAngleConstraint *SetEnabled(bool value){
		enabled=value;
		return this;
	}

	


	static float GetAngleOfParticlesWithLocalPositions( QParticle *particleA,QParticle *particleB,QParticle *particleC);

	/**
	 * By default, objects included in the physics engine are deleted by the destructors of the objects they belong to. When this flag is enabled, it indicates that this object should never be deleted by this engine. It is disabled by default, and it is recommended to keep it disabled. However, it can be used if needed for advanced purposes and integrations.
	 */
	bool manualDeletion=false;





};

#endif // QANGLECONSTRAINT_H
