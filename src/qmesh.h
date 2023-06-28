#ifndef QMESH_H
#define QMESH_H
#include <vector>
#include "cmath"
#include "qspring.h"
#include "qparticle.h"



using namespace std;

class QBody;

struct QMesh
{
public:
	enum CollisionBehaviors{
		CIRCLES,
		POLYGONS,
		POLYLINE
	};
protected:
	/**
	 * @brief Particles of the mesh (Positions are relative to the mesh)
	 */
	vector<QParticle*> particles=vector<QParticle*>();

	//General Properties
	QVector position=QVector::Zero();
	QVector globalPosition=QVector::Zero();
	vector<QSpring*> springs=vector<QSpring*>();
	vector<vector<QParticle*>> closedPolygons=vector<vector<QParticle*>>();
	float circumference=0.0f;
	QBody *ownerBody=nullptr;
	CollisionBehaviors collisionBehavior=CollisionBehaviors::CIRCLES;

	//Helper Methods
	void UpdateCollisionBehavior();

public:

	struct MeshData{
		vector<QVector> particlePositions;
		vector<float> particleRadValues;
		vector<bool> particleInternalValues;
		vector<pair<int,int>> springList;
		vector<pair<int,int>> internalSpringList;
		vector <vector<int>> closedPolygonList;
	};

	friend class QWorld;
	friend class QBody;

	QMesh();
	~QMesh();


	//General Get Methods
	QVector GetPosition(){
		return position;
	}
	QVector GetGlobalPosition(){
		return globalPosition;
	}
	float GetInitialArea(){
		float res=0.0f;
		for(auto poly:closedPolygons){
			res+=GetPolygonArea(poly,true);
		}
		for(auto particle:particles){
			if(particle->GetRadius()>0.5f){
				res+=particle->GetRadius()*particle->GetRadius();
			}
		}
		return res;
	}
	float GetInitialPolygonsArea(){
		float res=0.0f;
		for(auto poly:closedPolygons){
			res+=GetPolygonArea(poly,true);
		}
		return res;
	}

	float GetArea(){
		float res=0.0f;
		for(auto poly:closedPolygons){
			res+=GetPolygonArea(poly);
		}
		for(auto particle:particles){
			if(particle->GetRadius()>0.5f){
				res+=particle->GetRadius()*particle->GetRadius();
			}
		}
		return res;
	}
	float GetPolygonsArea(){
		float res=0.0f;
		for(auto poly:closedPolygons){
			res+=GetPolygonArea(poly);
		}

		return res;
	}

	float GetCircumference(){
		float res=0.0f;
		for(auto polygon:closedPolygons){
			for(int i=0;i<polygon.size();i++){
				QParticle *p=polygon[i];
				QParticle *np=polygon[(i+1)%polygon.size()];
				float length=(np->GetPosition()-p->GetPosition()).Length();
				res+=length;
			}
		}
//		for(auto spring:springs){
//			res+=(spring.GetParticleA()->GetGlobalPosition()-spring.GetParticleB()->GetGlobalPosition()).Length();
//		}
		return res;
	}
	QBody *GetOwnerBody(){
		return ownerBody;
	}

	CollisionBehaviors GetCollisionBehavior(){
		return collisionBehavior;
	}


	//General Set Methods
	QMesh* SetPosition(QVector value){
		position=value;
		return this;
	}
	QMesh* SetGlobalPosition(QVector value){
		globalPosition=value;
		return this;
	}





	//Particle Operations

	QMesh * AddParticle(QParticle *particle);

	QMesh * RemoveParticleAt(int index);
	QMesh * RemoveParticle(QParticle *particle);

	int GetParticleCount();
	QParticle *GetParticle(int index);

	//Closed Polygons Operations

	QMesh *AddClosedPolygon(vector<QParticle*> polygon);
	QMesh *RemoveClosedPolygonAt(int index);

	int GetClosedPolygonCount(){
		return closedPolygons.size();
	}
	vector<QParticle*> &GetClosedPolygon(int index){
		return closedPolygons[index];
	}
	QMesh *RemoveMatchingClosedPolygons(QParticle *particle);

	//Spring Operations
	QMesh *AddSpring(QSpring *spring);
	QMesh *RemoveSpring(QSpring *spring);
	QMesh *RemoveSpringAt(int index);
	QMesh *RemoveMatchingSprings(QParticle *particle);

	int GetSpringCount(){
		return springs.size();
	}
	QSpring *GetSpringAt(int index){
		return springs[index];
	}
	int GetSpringIndex(QSpring *spring){
		for(int i=0;i<springs.size();i++)
			if(springs[i]==spring)
				return i;
		return -1;
	}






	//Static Methods
	/**
	 * @brief It creates a mesh with only one particle.
	 * @param radius The radius of circle
	 * @param relativePosition The relative position of mesh
	 * @return QMesh
	 */
	static QMesh * CreateWithCircle(float radius,QVector relativePosition=QVector::Zero());
	/**
	 * @brief It creates a mesh with convex polygon shape.
	 * @param radius The radius of polygon
	 * @param sideCount The side count of polygon
	 * @param relativePosition The relative position of mesh
	 * @return QMesh
	 */
	static QMesh * CreateWithPolygon(float radius,int sideCount,QVector centerPosition=QVector::Zero(),int polarGrid=-1,bool enableSprings=true, bool enablePolygons=true,float particleRadius=0.5f);

	/**
	 * @brief It creates a mesh with convex rectangle shape
	 * @param size The size of rectangle
	 * @param relativePosition The relative position of mesh
	 * @param grid The grid of mesh
	 * @return QMesh
	 */
	static QMesh * CreateWithRect(QVector size,QVector centerPosition=QVector::Zero(),QVector grid=QVector::Zero(),bool enableSprings=true, bool enablePolygons=true,float particleRadius=0.5f);


	static QMesh * CreateWithMeshData(QMesh::MeshData &data,bool enableSprings=true, bool enablePolygons=true);





	/**
	 * @brief It gives the area of a polygon
	 * @param polygonPoints A points of the polygon
	 * @return float
	 */
	static float GetPolygonArea(vector<QParticle*> &polygonPoints,bool withLocalPositions=false);



	static bool CheckCollisionBehaviors(QMesh *meshA,QMesh * meshB,CollisionBehaviors firstBehavior,CollisionBehaviors secondBehavior );

	static MeshData GenerateRectangleMeshData(QVector size,QVector centerPosition=QVector::Zero(),QVector grid=QVector::Zero(),float particleRadius=0.5f);
	static MeshData GeneratePolygonMeshData(float radius, int sideCount, QVector centerPosition=QVector::Zero(),int polarGrid=-1,float particleRadius=0.5f);





};

#endif // QMESH_H
