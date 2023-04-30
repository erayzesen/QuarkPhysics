
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

#ifndef QRAYCAST_H
#define QRAYCAST_H
#include "qvector.h"
#include "qbody.h"

class QWorld;
/**
 * @brief QRaycast objects send a ray into the world and return collision results with body objects. You can create a constant raycast object that you can add to the world and update collision results at every physics step, or you can make instantaneous raycast calls at runtime using the QRaycast::RaycastTo static method. QWorld also provides methods for managing QRaycast objects.
 */
class QRaycast
{

protected:
	void UpdateContacts();

public:
	/** Creates a raycast. 
	 * @param position The position of the raycast. 
	 * @param rayVector The ray vector.
	 * @param enableContainingBodies Determines whether a body should be ignored in raycast collisions if the ray position is inside the shape representing the body in the world. If set to true, these objects will be ignored in raycast collisions.
	 */
	QRaycast(QVector position, QVector rayVector,bool enableContainingBodies=false);
	/**
	 * A contact struct for raycast collision results. It has some information about the ray collision. 
	 */
	struct Contact{
	public:
		/** The contact body of the ray collision.*/
		QBody *body;
		/** The contact position of the ray collision. */
		QVector position;
		/** The contact normal of the ray collision. */
		QVector normal;
		/** The distance between the ray position and the contact body. */
		float distance;
		Contact(QBody *body, QVector position, QVector normal,float distance): body(body), position(position),normal(normal),distance(distance){}

	};

	/** Sends a ray into the world with the given position and direction vector. Returns a collection of QRaycast::Contact containing collision information with body objects hit by the ray.
	 * @param world The world.
	 * @param rayPosition The position of the ray.
	 * @param rayVector The vector of the ray.
	 * @param collidableLayers The target layer bits.  
	 * @param enableContainingBodies Determines whether a body should be ignored in raycast collisions if the ray position is inside the shape representing the body in the world. If set to true, these objects will be ignored in raycast collisions.
	 * @return A collection of contacts as a result of the raycast operation. 
	 */
	static vector<QRaycast::Contact> RaycastTo(QWorld *world, QVector rayPosition,QVector rayVector, int collidableLayers=1,bool enableContainingBodies=false );





	//Get Methods
	/** Returns contact list of the raycast. */
	vector<QRaycast::Contact> *GetContacts();
	/** Returns the position of the raycast. */
	QVector GetPosition();
	/** Returns the rotation of the raycast. */
	float GetRotation();
	/** Returns the ray vector of the raycast. */
	QVector GetRayVector();
	/** Returns whether a body should be ignored in raycast collisions if the ray position is inside the shape representing the body in the world. If set to true, these objects will be ignored in raycast collisions. */ 
	bool GetEnabledContainingBodies();

	//Set Methods
	/** Sets the position of the raycast 
	 * @param val A position to set.
	 */
	QRaycast *SetPosition(QVector value);
	/** Sets the rotation of the raycast 
	 * @param value A rotation to set.
	 */
	QRaycast *SetRotation(float value);
	/** Sets the ray vector of the raycast 
	 * @param value A ray vector to set. 
	 */
	QRaycast *SetRayVector(QVector value);
	/** Sets whether a body should be ignored in raycast collisions if the ray position is inside the shape representing the body in the world. If set to true, these objects will be ignored in raycast collisions.
	 * @param value True or false. 
	 */
	QRaycast *SetEnabledContainingBodies(bool value);


	friend class QWorld;

private:
	QVector position=QVector::Zero();
	float rotation=0.0f;
	QVector ray=QVector::Zero();
	QVector rayOriginal=QVector::Zero();

	bool enabledContainingBodies=false;

	vector<QRaycast::Contact> contacts;

	QWorld *world;



	static vector<QBody*> GetPotentialBodies(QWorld *whichWorld,QVector rayPosition,QVector rayVector,int collidableLayers);
	static void RaycastToParticles(QBody *body, QMesh *mesh, QVector rayPosition, QVector rayVector, QVector rayUnit,QVector rayNormal, bool enableContainingBodies,vector<QRaycast::Contact> *contacts);
	static void RaycastToPolygon(QBody *body, QMesh *mesh, QVector rayPosition, QVector rayVector, QVector rayUnit,QVector rayNormal, bool enableContainingBodies,vector<QRaycast::Contact> *contacts);

	static bool SortContacts(const QRaycast::Contact contactA,const QRaycast::Contact contactB);

};

#endif // QRAYCAST_H
