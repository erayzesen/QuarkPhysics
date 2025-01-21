
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
#ifndef QAREABODY_H
#define QAREABODY_H
#include "qbody.h"
#include "qworld.h"
#include <functional>
#include <unordered_set>

class QManifold;
/**
 * @brief QAreaBody objects are objects that don't respond to collisions or receive any response from them, but only report collisions. An operation is not applied for them to move during physics steps, they are stationary. Unlike other body types, they have two event listeners named OnCollisionEnter and OnCollisionExit.
 */
class QAreaBody : public QBody{
	unordered_set<QBody*> bodies;
	void AddCollidedBody(QBody *body);
	void CheckBodies();
	bool gravityFree=false;
	QVector linearForceToApply=QVector::Zero();
public:
	QAreaBody();
	/** This event listener is triggered when any object enters the collision list of a QAreaBody for the first time.
	 * @param collidedBody The body entered to the collision list.
	 */
	virtual void OnCollisionEnter(QBody *collidedBody){};
	/** This event listener is triggered when any object exits the collision list of a QAreaBody for the first time.
	 * @param collidedBody The body exited from the collision list.
	 */
	virtual void OnCollisionExit(QBody *collidedBody){};
	/** This is the event listener callback function for the OnCollisionEnter event.
	 * @param areaBody An area body.
	 * @param collidedBody The collided body.
	 */
	std::function<void(QAreaBody *areaBody,QBody* collidedBody)> CollisionEnterEventListener;
	/** This is the event listener callback function for the OnCollisionExit event.
	 * @param areaBody An area body.
	 * @param collidedBody The collided body.
	 */
	std::function<void(QAreaBody *areaBody,QBody* collidedBody)> CollisionExitEventListener;

	/**
	 * Returns whether the option to exempt objects entering the area from gravity is enabled. If enabled, it will disable both the globally defined gravity and the body-specific gravity applied to physics bodies entering the area.
	 */
	bool GetGravityFreeEnabled(){
		return gravityFree;
	}
	/**
	 * Returns the amount of linear force to be applied to objects entering the area. If a force vector is defined, objects entering the area will be continuously subjected to this force.
	 */
	QVector GetLinearForceToApply(){
		return linearForceToApply;
	}

	/**
	 * Sets whether the option to exempt objects entering the area from gravity is enabled. If enabled, it will disable both the globally defined gravity and the body-specific gravity applied to physics bodies entering the area.
	 * @param value A bool vector to set
	 * @return A pointer to the body itself.
	 */
	QAreaBody* SetGravityFreeEnabled(bool value){
		gravityFree=value;
		for (auto body : bodies){
			body->ignoreGravity=gravityFree;
		}
		return this;
	}
	/**
	 * Sets the amount of linear force to be applied to objects entering the area. If a force vector is defined, objects entering the area will be continuously subjected to this force.
	 * @param value A force vector to set.
	 * @return A pointer to the body itself.
	 */
	QAreaBody* SetLinearForceToApply(QVector value){
		linearForceToApply=value;
		return this;
	}

	

	


	friend class QManifold;
	friend class QWorld;

};


#endif //QAREABODY_H
