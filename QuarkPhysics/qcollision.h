
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

#ifndef QCOLLISION_H
#define QCOLLISION_H
#include <iostream>
#include <vector>
#include "qparticle.h"

class QWorld;
using namespace std;


/** @brief The QCollision class performs all collision detection operations. The relevant methods return contact data from the collision tests.
 */
class QCollision
{
public:
	QCollision();;


	/** @brief Contains all the contact information required to resolve collisions. */
	struct Contact{
	public:
		/** The contact position of the collision */
		QVector position;
		/** The incident particle of the collision */
		QParticle *particle;
		/** The normal of the collision */
		QVector normal;
		/** The penetration of the collision */
		float penetration;
		/** The particle list of the reference face in the collision */
		vector<QParticle*> referenceParticles;
		/** Determines whether the contact is solved. */
		bool solved;
		Contact(QParticle *particle,QVector position,QVector normal,float penetration,vector<QParticle*> referenceParticles ){
			this->particle=particle;
			this->position=position;
			this->normal=normal;
			this->penetration=penetration;
			this->referenceParticles=referenceParticles;
			this->solved=false;
		}



	};



	struct Project{
		public:
			float min;
			float max;
			int minIndex;
			int maxIndex;
			Project(float projectMin,int minPointIndex,float projectMax,int maxPointIndex){
				min=projectMin;
				max=projectMax;
				minIndex=minPointIndex;
				maxIndex=maxPointIndex;
			}
			float Overlap(const Project other) const{
				float penetration=0.0f;
				if(other.min<min){
					penetration=min-other.max;
				}else{
					penetration=other.min-max;
				}
				return penetration;
			}

	};

	//Collision Methods
	/** Checks collisions between polygons. 
	 * @param particlesA A collection of particles that make up a polygon.
	 * @param particlesB Another collection of particles that make up a polygon.
	 * @param contacts A collection where collision contact information will be stored.
	 */
	static void PolygonAndPolygon(vector<QParticle*> &particlesA,vector<QParticle*> &particlesB,vector<QCollision::Contact> &contacts);
	/** Checks collisions between circle(s) and polygon. 
	 * @param particlesA A collection of particles representing one or more circles, each having a radius.
	 * @param particlesB A collection of particles that make up a polygon.
	 * @param contacts A collection where collision contact information will be stored.
	 */
	static void CircleAndPolygon(vector<QParticle*> &circleParticles,vector<QParticle*> &polygonParticles,vector<QCollision::Contact> &contacts);
	/** Checks collisions between circle(s) and circle(s). 
	 * @param particlesA A collection of particles representing one or more circles, each having a radius.
	 * @param particlesB Another collection of particles representing one or more circles, each having a radius.
	 * @param contacts A collection where collision contact information will be stored.
	 */
	static void CircleAndCircle(vector<QParticle*> &particlesA,vector<QParticle*> &particlesB,vector<QCollision::Contact> &contacts);
	/** Checks collisions between polyline and polygon. 
	 * @param particlesA A collection of particles that make up a polyline.
	 * @param particlesB Another collection of particles that make up a polygon.
	 * @param contacts A collection where collision contact information will be stored.
	 * \note To check collisions between two polylines, call this method twice with the polylines swapped as arguments.
	 * \note "Polyline" is commonly used to define polygons for softbody objects. In contrast to a solid and filled polygon , you can think of a polyline as a polygon shape formed by a  connected and knotted rope. If a polygon-shaped rope is deformed, it may not always remain a polygon, it can overlap and its corners can merge. Collision testing for a polyline is performed based on these assumptions
	 */
	static void PolylineAndPolygon(vector<QParticle*> &polylineParticles,vector<QParticle*> &polygonParticles,vector<QCollision::Contact> &contacts);
	/** Checks collisions between polyline and circle(s). 
	 * @param polylineParticles A collection of particles that make up a polyline.
	 * @param circleParticles A collection of particles representing one or more circles, each having a radius.
	 * @param contacts A collection where collision contact information will be stored.
	 * \note "Polyline" is commonly used to define polygons for softbody objects. In contrast to a solid and filled polygon , you can think of a polyline as a polygon shape formed by a  connected and knotted rope. If a polygon-shaped rope is deformed, it may not always remain a polygon, it can overlap and its corners can merge. Collision testing for a polyline is performed based on these assumptions
	 */
	static void PolylineAndCircle(vector<QParticle*> &polylineParticles,vector<QParticle*> &circleParticles,vector<QCollision::Contact> &contacts);

	static void PolyLineSelfCollision(vector<QParticle*> polylineParticles,vector<QCollision::Contact> &contacts);


	//Geometry Helper Methods
	/** Checks intersection between lines.
	 * @param d1A The point A of the d1 line.
	 * @param d1B The point B of the d1 line.
	 * @param d2A The point A of the d2 line.
	 * @param d2B The point B of the d2 line.
	 * @return An intersection position. If the intersection doesn't exist, returns QVector::NaN. 
	 */
	static QVector LineIntersectionLine(QVector d1A,QVector d1B, QVector d2A, QVector d2B);
	/** Checks whether a specified point is inside the specified polygon.
	 * @param point A point to check.
	 * @param polygon A collection of particles that make up a polygon.
	 * @return Returns true if the point is inside the polygon, false otherwise.
	 */
	static bool PointInPolygon(QVector &point, vector<QParticle*> &polygon );
	/** Checks whether a specified point is inside the specified polygon.
	 * @param point A point to check.
	 * @param polygon A collection of vectors that make up a polygon.
	 * @return Returns true if the point is inside the polygon, false otherwise.
	 */
	static bool PointInPolygon(QVector &point, vector<QVector> &polygon );
	/** Checks whether a specified point is inside the specified polygon.
	 * @param point A point to check.
	 * @param polygon A collection of particles that make up a polygon.
	 * @return Returns true if the point is inside the polygon, false otherwise.
	 */
	static bool PointInPolygon2(QVector point, vector<QParticle*> &polygon );
private:

	//Collision Helper Methods
	static void ClipContactParticles(QParticle *referenceParticles[], QParticle *incidentParticles[], vector<QCollision::Contact> &contacts );
	static Project ProjectToAxis(QVector &normal,vector<QParticle*> &polygon);



};



#endif // QCOLLISION_H
