#ifndef QCOLLISION_H
#define QCOLLISION_H
#include <iostream>
#include <vector>
#include "qparticle.h"

class QWorld;

using namespace std;
class QCollision
{
public:
	QCollision();;


	struct Contact{
	public:
		QVector position;
		QParticle *particle;
		QVector normal;
		float penetration;
		vector<QParticle*> referenceParticles;
		Contact(QParticle *particle,QVector position,QVector normal,float penetration,vector<QParticle*> referenceParticles ){
			this->particle=particle;
			this->position=position;
			this->normal=normal;
			this->penetration=penetration;
			this->referenceParticles=referenceParticles;
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

	static void PolygonAndPolygon(vector<QParticle*> &particlesA,vector<QParticle*> &particlesB,vector<QCollision::Contact> &contacts,QWorld *world);
	static void CircleAndPolygon(vector<QParticle*> &circleParticles,vector<QParticle*> &polygonParticles,vector<QCollision::Contact> &contacts);
	static void CircleAndCircle(vector<QParticle*> &particlesA,vector<QParticle*> &particlesB,vector<QCollision::Contact> &contacts);
	static void PolylineAndPolygon(vector<QParticle*> &polylineParticles,vector<QParticle*> &polygonParticles,vector<QCollision::Contact> &contacts);
	static void PolylineAndCircle(vector<QParticle*> &polylineParticles,vector<QParticle*> &circleParticles,vector<QCollision::Contact> &contacts);

	//Collision Helper Methods

	static void ClipContactParticles(QParticle *referenceParticles[], QParticle *incidentParticles[], vector<QCollision::Contact> &contacts );
	static Project ProjectToAxis(QVector &normal,vector<QParticle*> &polygon);

	//Geometry Helper Methods

	static QVector LineIntersectionLine(QVector d1A,QVector d1B, QVector d2A, QVector d2B);
	static bool PointInPolygon(QVector &point, vector<QParticle*> &polygon );
	static bool PointInPolygon(QVector &point, vector<QVector> &polygon );
	static bool PointInPolygon2(QVector point, vector<QParticle*> &polygon );



};



#endif // QCOLLISION_H
