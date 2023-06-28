#ifndef QWORLD_H
#define QWORLD_H
#include <iostream>
#include <map>
#include <vector>
#include "qbody.h"
#include "qjoint.h"
#include "qvector.h"
#include "qmanifold.h"
#include "qgizmos.h"
#include "qbroadphase.h"
#include "qcollision.h"
#include "qrigidbody.h"
#include "qsoftbody.h"
#include "qraycast.h"
#include "qjoint.h"


/**
 *@brief QWorld is a world of the physics simulation.
 *QWorld is responsible for simulation steps and elements.
 */
using namespace std;
class QWorld{

	struct bodyPairHash {
		size_t operator()(const std::pair<QBody*, QBody*> &p) const {
			 return std::hash<QBody*>()(p.first) + std::hash<QBody*>()(p.second);
		}
	};

	struct bodyPairEqual {
		bool operator()(const std::pair<QBody*, QBody*> &p1, const std::pair<QBody*, QBody*> &p2) const {
			return ( (p1.first == p2.first && p1.second == p2.second) ) ;

		}
	};
protected:
	//Collections
	vector<QBody*> bodies=vector<QBody*>();

	vector <QJoint*> joints=vector<QJoint*>();

	vector <QSpring*> springs=vector<QSpring*>();

	vector<QRaycast*> raycasts=vector<QRaycast*>();

	vector <QGizmo*> gizmos=vector<QGizmo*>();

	vector<vector<QBody*> > sleepingIslands=vector<vector<QBody*> >();
	unordered_set<pair<QBody*, QBody*>, bodyPairHash,bodyPairEqual> collisionExceptions;

	QBroadPhase broadPhase=QBroadPhase(QVector(128.0f,128.0f));

	vector<QManifold> manifolds;


	//Physics World Properties

	QVector gravity=QVector(0.0f,0.2f);
	bool enableSleeping=true;
	bool enableBroadphase=true;
	int iteration=4;

	//Infographic Features
	int debugAABBTestCount=0; //aabb test method call count
	int debugCollisionTestCount=0; // any collision method call count

	void ClearGizmos();
	void ClearBodies();


	//Collisions, Dynamic Colliders Islands and Sleeping Feature
	void CreateIslands(QBody *body, vector<QBody*> &bodyList, vector<QBody*> &island);
	void GenerateIslands(vector<QBody*> &bodyList, vector<vector<QBody*>> &islandList);
	static bool SortBodies(const QBody *bodyA,const QBody *bodyB);
	static bool SortBodiesVertical(const QBody *bodyA,const QBody *bodyB);
	void GetCollisionPairs(vector<QBody *> &bodyList,vector<pair<QBody *, QBody *> > *resList);

	void CreateIslands(QBody &body, vector<QBody*> island);
	vector<vector<QBody>> GenerateIslands(vector<QBody> bodyList );

	//Constraints
	void UpdateConstraints();



public:

	inline static float MAX_WORLD_SIZE=99999.0f;

	//General Get Methods

	QVector GetGravity(){
		return gravity;
	}
	bool GetEnableSleeping(){
		return enableSleeping;
	}
	bool GetEnableBroadphase(){
		return enableBroadphase;
	}
	int GetIterationCount(){
		return iteration;
	}

	//General Set Methods
	QWorld *SetGravity(QVector value){
		gravity=value;
		return this;
	}
	QWorld *SetEnableSleeping(bool value){
		enableSleeping=value;
		return this;
	}
	QWorld *SetEnableBroadphase(bool value){
		enableBroadphase=value;
		return this;
	}
	QWorld *SetIterationCount(int value){
		iteration=value;
		return this;
	}

	//Methods
	void Update();



	///It performs the collision of 2 objects.
	static vector<QCollision::Contact> GetCollisions(QBody *bodyA, QBody *bodyB);


	/**It adds a body in the world
	 * @param body A body to be added
	 */
	QWorld *AddBody(QBody *body);
	/**It adds a body group in the world
	 * @param bodyGroup A body collection to be added
	 */
	QWorld *AddBodyGroup(const vector<QBody*> &bodyGroup);
	QWorld *RemoveBody(QBody *body);
	QWorld *RemoveBodyAt(int index);

	int GetBodyCount(){
		return bodies.size();
	}
	QBody *GetBodyAt(int index){
		return bodies[index];
	}
	int GetBodyIndex(QBody *body){
		for(int i=0;i<bodies.size();i++)
			if(bodies[i]==body)
				return i;
		return -1;
	}
	vector<QBody*> GetBodiesHitByPoint(QVector point,int maxBodyCount=1,bool onlyRigidBodies=true,int layersBit=-1);
	vector<QParticle*> GetParticlesCloseToPoint(QVector point,float distance,int maxParticleCount=1,bool exceptRigidBodies=true,int layersBit=-1);


	//Joint Operations
	QWorld *AddJoint(QJoint *joint);
	QWorld *RemoveJoint(QJoint *joint);
	QWorld *RemoveJointAt(int index);
	QWorld *RemoveMatchingJoints(QBody *body);

	int GetJointCount(){
		return joints.size();
	}
	QJoint *GetJointAt(int index){
		return joints[index];
	}
	int GetJointIndex(QJoint *joint){
		for(int i=0;i<joints.size();i++)
			if(joints[i]==joint)
				return i;
		return -1;
	}

	//Spring Operations
	QWorld *AddSpring(QSpring *spring);
	QWorld *RemoveSpring(QSpring *spring);
	QWorld *RemoveSpringAt(int index);
	QWorld *RemoveMatchingSprings(QBody *body);
	QWorld *RemoveMatchingSprings(QParticle *particle);

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

	//Raycast Operations
	QWorld *AddRaycast(QRaycast * raycast);
	QWorld *RemoveRaycast(QRaycast *raycast);
	QWorld *RemoveRaycastAt(int index);

	int GetRaycastCount(){
		return raycasts.size();
	}
	QRaycast *GetRaycastAt(int index){
		return raycasts[index];
	}
	int GetRaycastIndex(QRaycast *spring){
		for(int i=0;i<raycasts.size();i++)
			if(raycasts[i]==spring)
				return i;
		return -1;
	}

	//Gizmos Operations
	vector<QGizmo*> *GetGizmos(){
		return &gizmos;
	};

	//Collision Exceptions

	QWorld *AddCollisionException(QBody *bodyA, QBody *bodyB);
	QWorld *RemoveCollisionException(QBody *bodyA, QBody *bodyB);
	QWorld *RemoveMatchingCollisionException(QBody *body);
	bool CheckCollisionException(QBody *bodyA, QBody *bodyB);

	QWorld();
	~QWorld();

	QWorld *ClearJoints();
	QWorld *ClearSprings();
	QWorld *ClearRaycasts();
	QWorld *ClearWorld();

	friend class QCollision;
	friend class QManifold;



};

#endif // QWORLD_H
