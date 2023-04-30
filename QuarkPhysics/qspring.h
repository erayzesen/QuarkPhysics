
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

#ifndef QSPRING_H
#define QSPRING_H
#include "qparticle.h"

class QMesh;
/**
 *@brief You can apply distance constraints between 2 particles using the QSpring. The physics engine uses QSpring to impose distance constraints between particles in objects already simulated using mass-spring models (e.g. QSoftBody objects). However, if the user wants, they can apply specific distance constraints between any 2 particles using QSpring. QWorld also provides methods to manage QSpring objects.
 *\note You can think of QSpring objects as similar to QJoint, which provides special distance constraints for rigid body objects in particle-based simulations. You can also use QSpring to connect objects with each other, but the constraints are enforced between particles.
 *\warning QSpring will not work correctly on objects that are not simulated with particle dynamics, such as rigid bodies whose positions are fixed by transformations. QSpring relies on the ability to change the positions of particles with freedom of movement to apply constraints correctly.
 */
class QSpring
{
	float rigidity=1.0f;
	QParticle *pA;
	QParticle *pB;
	float length;
	bool isInternal=false;
	bool enabled=true;
public:
	/**
	 * Creates a spring between two particles. But auto calculates length with the distance between two particles.  
	 * @param particleA A particle in the world.
	 * @param particleB Another particle in the world.
	 * @param internal Determines whether the spring connection between two particles is an internal connection within an object. It can usually be set to false. This setting is important for soft body objects with the volume preserving option enabled, where internal spring connections are restricted in different ways. 
	 */
	QSpring(QParticle *particleA,QParticle *particleB,bool internal=false);
	/**
	 * Creates a spring between two particles with a specified length.  
	 * @param particleA A particle in the world.
	 * @param particleB Another particle in the world.
	 * @param length The distance between the two particles.
	 * @param internal Determines whether the spring connection between two particles is an internal connection within an object.It can usually be set to false. This setting is important for soft body objects with the volume preserving option enabled, where internal spring connections are restricted in different ways. 
	 */
	QSpring(QParticle *particleA,QParticle *particleB,float length,bool internal=false);



	/**
	 * Applies spring constraints and updates particle positions.
	 * @param rigidity The rigidity of the constraint. The rigidity must be a value between 0.0 and 1.0. 
	 * @param internalsException It is usually set to false. However, if set to true, it pays attention to internal particle connections and applies the constraints accordingly. This setting is important for soft body objects with the volume preserving option enabled.
	 * \Note This method is virtual and users can implement custom spring update methods in an interited class of QSpring. 
	 */
	virtual void Update(float rigidity,bool internalsException);

	//Get Methods
	/** Returns particleA of the spring. */
	QParticle *GetParticleA(){
		return pA;
	}
	/** Returns particleB of the spring. */
	QParticle *GetParticleB(){
		return pB;
	}
	/** Returns the length of the spring. */
	float GetLength(){
		return length;
	}
	/** Returns whether the spring is internal. */ 
	bool GetIsInternal(){
		return isInternal;
	}
	/** Returns the rigidity of the spring. */
	float GetRigidity(){
		return rigidity;
	}
	/** Returns whether the spring is enabled. */
	bool GetEnabled(){
		return enabled;
	}

	//Set Methods
	/** Sets particleA of the spring.
	 * @param particle A pointer to the particle.
	 * @return A pointer to the spring itself.
	 */
	QSpring *SetParticleA(QParticle *particle){
		pA=particle;
		return this;
	}
	/** Sets particleB of the spring.
	 * @param particle A pointer to the particle.
	 * @return A pointer to the spring itself.
	 */
	QSpring *SetParticleB(QParticle *particle){
		pB=particle;
		return this;
	}
	/** Sets the length of the spring.
	 * @param length The length to set.
	 * @return A pointer to the spring itself.
	 */
	QSpring *SetLength(float length){
		this->length=length;
		return this;
	}
	/** Sets whether the spring is internal.
	 * @param value True or false.
	 * @return A pointer to the spring itself.
	 */
	QSpring *SetIsInternal(bool value){
		isInternal=value;
		return this;
	}
	/** Sets the rigidity of the spring.
	 * @param rigidity The rigidity value to set.It must be a value between 0.0 and 1.0.
	 * @return A pointer to the spring itself.
	 */
	QSpring *SetRigidity(float rigidity){
		this->rigidity=rigidity;
		return this;
	}
	/** Sets whether the spring is enabled. 
	 * @param value True or false.
	 * @return A pointer to the spring itself.
	 */
	QSpring *SetEnabled(bool value){
		enabled=value;
		return this;
	}





};

#endif // QSPRING_H
