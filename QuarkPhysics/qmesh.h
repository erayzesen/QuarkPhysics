
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

#ifndef QMESH_H
#define QMESH_H
#include <vector>
#include "cmath"
#include "qspring.h"
#include "qparticle.h"
#include "qangleconstraint.h"
#include "json/json.hpp"
#include "fstream"
#include "qmath_utils.h"

using json =nlohmann::json;

using namespace std;

class QBody;
/**
 *@brief Every QBody object requires meshes. In other traditional physics engines, the term 'shape' is used instead. However, in Quark Physics, meshes do not only contain information about primitive shapes. A QMesh includes collision shapes, collision behaviors, internal and external spring connections, particle informations, and necessary collective information for rendering.
 * In the QMesh class, there are methods to quickly create primitive types such as circles, rectangles, and polygons that are suitable for the types and needs of body objects. However, QMesh objects are created with a struct called MeshData. Therefore, it is also possible to create many complex mesh examples for body types.
 */
struct QMesh
{
public:
	enum CollisionBehaviors{
		CIRCLES,
		POLYGONS,
		POLYLINE
	};
protected:
	vector<QParticle*> particles=vector<QParticle*>();

	//General Properties
	QVector position=QVector::Zero();
	QVector globalPosition=QVector::Zero();
	float rotation=0.0f;
	float globalRotation=0.0f;
	vector<QSpring*> springs=vector<QSpring*>();
	vector<QAngleConstraint*> angleConstraints=vector<QAngleConstraint*>();
	vector <QParticle*> polygon=vector<QParticle*>();
	vector<vector<QParticle*>> subConvexPolygons=vector<vector<QParticle*>>();
	float circumference=0.0f;
	QBody *ownerBody=nullptr;
	CollisionBehaviors collisionBehavior=CollisionBehaviors::CIRCLES;
	vector<vector<int>> UVMaps=vector<vector<int>>();
	bool disablePolygonForCollisions=false;

	bool collisionBehaviorNeedsUpdate=false;

	//Helper Methods
	void UpdateCollisionBehavior();

	//Polygon Properties
	vector<float> lastPolygonCornerAngles;
	float minAngleConstraintOfPolygon=M_PI*0.3;

	//Polygon Methods
	void UpdateSubConvexPolygons(bool majorUpdate=true);
	void ApplyAngleConstraintsToPolygon();
	bool CheckIsPolygonConcave(vector<QParticle*> polygonParticles);
	static bool CheckIsReflex(QVector pA,QVector pB, QVector pC);
	static bool CheckIsReflex(int indexA,int indexB, int indexC, vector<QParticle*> polygonParticles);
	static void TriangulatePolygon(vector<QParticle*> &polygonParticles,vector<vector<int>> &triangles);
	static void DecompositePolygon(vector<QParticle*> &polygonParticles,vector<vector<QParticle*>> &polygons);
	static void DecompositePolygon2(vector<QParticle*> &polygonParticles,vector<vector<QParticle*>> &polygons);
	bool subConvexPolygonsNeedsUpdate=false;

public:
	/** The data struct of the mesh. 
	 */
	struct MeshData{
		/** The collection of local positions of particles. */
		vector<QVector> particlePositions;
		/** The collection of radius values of particles. */
		vector<float> particleRadValues;
		/** The collection of boolean values indicating whether a particle is internal. */
		vector<bool> particleInternalValues;
		/** The collection of boolean values indicating whether a particle is enabled.  */
		vector<bool> particleEnabledValues;
		/** The collection of boolean values indicating whether a particle is lazy.  */
		vector<bool> particleLazyValues;
		/** The collection of integer pairs to define springs. 
		 * The integer values define the indices of particles in the particlePositions collection.  
		 * */
		vector<pair<int,int>> springList;
		/** The collection of integer pairs to define internal springs. 
		 * Internal springs are important for the some mass spring simulation configrations.
		 * The integer values define the indices of particles in the particlePositions collection.  
		 * */
		vector<pair<int,int>> internalSpringList;
		/** The polygon collection containing the index collection of the polygons.
		 * Polygons are important to define polygon colliders of the mesh.
		 * The particle orders should be clockwise.
		 * The integer values define the indices of particles in the particlePositions collection.  
		 * */
		vector<int> polygon;


		/** Contains UV maps. UV polygons can be defined using the index numbers of the
		 *  particlePositions collection. Each polygon must have at least 3 points.
		 *  Meshes created with CreateWithRect() or CreateWithPolygon() generate
		 *  UV maps by subdividing the mesh into triangles.
		 */
		vector<vector<int>> UVMaps;

		/** The position of the mesh */
		QVector position=QVector::Zero();

		/** The rotation of the mesh */
		float rotation=0.0f;
		
	};

	friend class QWorld;
	friend class QBody;
	friend class QRigidBody;
	friend class QSoftBody;
	friend class QRaycast;
	friend class QCollision;
	friend class QParticle;

	/** Creates a mesh. */
	QMesh();
	~QMesh();


	//General Get Methods
	/** Returns the local position of the mesh. */
	QVector GetPosition(){
		return position;
	}
	/** Returns the global position of the mesh. */
	QVector GetGlobalPosition(){
		return globalPosition;
	}
	/** Returns the local rotation of the mesh. */
	float GetRotation(){
		return rotation;
	}
	/** Returns the global rotation of the mesh. */
	float GetGlobalRotation(){
		return globalRotation;
	}
	/** Returns the total area of the mesh with local positions of particles */
	float GetInitialArea(){
		float res=0.0f;
		for(size_t n=0;n<GetSubConvexPolygonCount();n++){

			res+=GetPolygonArea(GetSubConvexPolygonAt(n),true);
		}
		for(auto particle:particles){
			if(particle->GetRadius()>0.5f){
				res+=particle->GetRadius()*particle->GetRadius();
			}
		}
		return res;
	}
	/** Returns the total polygon area of the mesh with local positions of particles */
	float GetInitialPolygonsArea(){
		float res=0.0f;
		for(size_t n=0;n<GetSubConvexPolygonCount();n++){
			res+=GetPolygonArea(GetSubConvexPolygonAt(n),true);
		}
		return res;
	}
	/** Returns total area of the mesh with global positions of particles */

	float GetArea(){
		float res=0.0f;
		for(size_t n=0;n<GetSubConvexPolygonCount();n++){
			res+=GetPolygonArea(GetSubConvexPolygonAt(n));
		}
		for(auto particle:particles){
			if(particle->GetRadius()>0.5f){
				res+=particle->GetRadius()*particle->GetRadius();
			}
		}
		return res;
	}
	/** Returns total polygon area of the mesh with global positions of particles */
	float GetPolygonsArea(){
		float res=0.0f;
		for(size_t n=0;n<GetSubConvexPolygonCount();n++){
			res+=GetPolygonArea(GetSubConvexPolygonAt(n));
		}

		return res;
	}

	/** Returns total circumference of all polygons of the mesh (Calculates with local positions of particles) */
	float GetCircumference(){
		float res=0.0f;
		for(size_t n=0;n<GetSubConvexPolygonCount();n++){
			auto polygon=GetSubConvexPolygonAt(n);
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
	/** Returns owner body of the mesh. 
	 * Owner body is the body in which the mesh is appointed. 
	 */
	QBody *GetOwnerBody(){
		return ownerBody;
	}
	/** Returns collision behaviors of the mesh. Collision behaviors can be circles, polygons, polylines.  
	 * The behaviors feature is important to determine collision methods for the mesh in the runtime.
	 * */
	CollisionBehaviors GetCollisionBehavior(){
		if(collisionBehaviorNeedsUpdate){
			UpdateCollisionBehavior();
			collisionBehaviorNeedsUpdate=false;
		}
		return collisionBehavior;
	}
	/** Returns whether the polygon is disabled for collisions.

		The polygon defined for the mesh is disabled during collision behavior determination and
		is not used in collisions. As a result, collisions are handled by the particles.
		The polygon can optionally be used for rendering purposes and other external
		purposes in the related simulation.  */
	bool GetPolygonForCollisionsDisabled(){
		return disablePolygonForCollisions;
	}


	//General Set Methods
	/** Sets the local position of the mesh 
	 * @param value The local position value to set.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh* SetPosition(QVector value){
		position=value;
		return this;
	}
	/** Sets the global position of the mesh 
	 * @param value The global position value to set.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh* SetGlobalPosition(QVector value){
		globalPosition=value;
		return this;
	}
	/** Sets the rotation of the mesh 
	 * @param value The rotation value to set.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh *SetRotation(float value){
		rotation=value;
		return this;
	}

	/** Sets whether the polygon is disabled for collisions.
	 * The polygon defined for the mesh is disabled during collision behavior determination and
	 * is not used in collisions. As a result, collisions are handled by the particles.
	 * The polygon can optionally be used for rendering purposes and other external
	 * purposes in the related simulation. 
	 * @param value The boolean value to set.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh *SetPolygonForCollisionsDisabled(bool value){
		disablePolygonForCollisions=value;
		collisionBehaviorNeedsUpdate=true;
		return this;
	}
	




	//Particle Operations
	
	/** Adds a particle to the mesh
	 * @param particle A particle to be added.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh * AddParticle(QParticle *particle);

	/** Removes the particle from the mesh at the specified index.
	 * @param particle The index of particle to be removed.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh * RemoveParticleAt(int index);
	/** Removes a particle from the mesh
	 * @param particle A particle to be removed.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh * RemoveParticle(QParticle *particle);
	/** Returns the total particle count in the mesh.
	 */
	int GetParticleCount();
	/** Returns a particle at the specified index
	 * @param particle The index of particle to get.
	 */
	QParticle *GetParticleAt(int index);

	/** Returns the index of the specified particle.
	 * @param particle A particle in the mesh.
	 * @return If the particle is found, it returns the index value, otherwise it returns -1. 
	 */
	int GetParticleIndex(QParticle *particle){
		for(int i=0;i<particles.size();i++){
			if(particles[i]==particle){
				return i;
			}
		}
		return -1;
	}




	//Polygon Operations

	/** Sets a polygon to the mesh
	 * @param polygonParticles A particle pointers collection of the polygon.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh * SetPolygon(vector<QParticle *> polygonParticles); 

	/** Adds a particle of the mesh to the polygon. If you want to add a particle to the end of the polygon, set the position value as -1. 
	 * @param particle A pointer to a particle
	 * @param position A position index. Default value is -1, indicating the end of the polygon.
	 * @return QMesh* A pointer to mesh itself.
	 */

	QMesh * AddParticleToPolygon(QParticle * particle, int position=-1);

	/** Removes a particle from to the polygon.  
	 * @param particle A pointer to a particle
	 * @return QMesh* A pointer to mesh itself.
	 * @note A polygon requires at least 3 particles. Please check the number of particles in the polygon before removing any particle from it.
	 */

	QMesh *RemoveParticleFromPolygon(QParticle * particle);

	/** Removes a particle from to the polygon at the specified index. 
	 * @param index An index value
	 * @return QMesh* A pointer to mesh itself.
	 * @note A polygon requires at least 3 particles. Please check the number of particles in the polygon before removing any particle from it.
	 */

	QMesh * RemoveParticleFromPolygonAt(int index);

	/** Removes the polygon from the mesh.
	 * @return QMesh* A pointer to mesh itself.
	 */

	QMesh * RemovePolygon();

	/** Returns the  total particle count of the polygon  */

	int GetPolygonParticleCount();

	/** Returns a particle of the polygon at the specified index  
	 * @param index An index value
	 * @return QParticle* A pointer to the particle.
	*/

	QParticle *GetParticleFromPolygon(int index);


	/** Returns the minimum angle for the angle constraints of the polygon. If the constraints are disabled, the value will be 0. 
	 * The default value is pi * 0.1.
	 * @return The minimum angle value in radians of the angle constraints of the polygon.
	*/
	float  GetMinAngleConstraintOfPolygon(){
		return minAngleConstraintOfPolygon;
	}

	/** Sets the minimum angle for the angle constraints of the polygon. If the value is 0, it means constraints are disabled. 
	 * The default value is pi * 0.1.
	 * @param radian The minimum angle value in radians.
	 * @return QMesh* A pointer to mesh itself.
	*/
	QMesh *SetMinAngleConstraintOfPolygon(float radian){
		minAngleConstraintOfPolygon=radian;
		return this;
	}

	

	/** Returns the total sub polygon count in the mesh.
	 */
	int GetSubConvexPolygonCount(){
		if (subConvexPolygonsNeedsUpdate==true){
			UpdateSubConvexPolygons();
			subConvexPolygonsNeedsUpdate=false;
		}
		return subConvexPolygons.size();
	}
	/** Returns sub polygon at the specified index
	 * @param index The index of the sub polygon to get.
	 */
	vector<QParticle*> &GetSubConvexPolygonAt(int index){
		if (subConvexPolygonsNeedsUpdate==true){
			UpdateSubConvexPolygons();
			subConvexPolygonsNeedsUpdate=false;
		}
		return subConvexPolygons[index];
	}
	

	//Spring Operations
	
	/** Adds a spring to the mesh
	 * @param spring A spring to be added.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh *AddSpring(QSpring *spring);
	/** Removes a spring to the mesh
	 * @param spring A spring to be removed.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh *RemoveSpring(QSpring *spring);
	/** Removes a spring to the mesh at the specified index.
	 * @param index The index of the spring to be removed.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh *RemoveSpringAt(int index);
	/** Removes the springs that contain the specified particle.
	 * @param particle A particle to be matched.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh *RemoveMatchingSprings(QParticle *particle);
	/** Returns the total spring count in the mesh.
	 */
	int GetSpringCount(){
		return springs.size();
	}
	/** Returns the spring at the specified index.
	 * @param index The index of the spring to be get.
	 */
	QSpring *GetSpringAt(int index){
		return springs[index];
	}
	/** Returns the index of the specified spring.
	 * @param spring A spring in the mesh.
	 * @return If the spring is found, it returns the index value, otherwise it returns -1.
	 */
	int GetSpringIndex(QSpring *spring){
		for(int i=0;i<springs.size();i++)
			if(springs[i]==spring)
				return i;
		return -1;
	}

	

	//UV Operations

	/** Returns the count of the UV maps .
	 */
	int GetUVMapCount(){
		return UVMaps.size();
	};

	/** Returns the UV map at the specified index.
	 * @param index The index of the UV map to be get.
	 */
	vector<int> GetUVMapAt(int index){
		return UVMaps[index];
	};

	/** Adds a UV map to the mesh
	 * @param map A particle index collection to be added.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh * AddUVMap(vector<int> map);

	/** Removes the springs that contain the specified particle.
	 * @param index The index of the UV map to be removed.
	 * @return QMesh* A pointer to mesh itself.
	 */

	QMesh * RemoveUVMapAt(int index);

	/** Removes all UV maps of the mesh.
	 * @return QMesh* A pointer to mesh itself.
	 */

	QMesh * ClearUVMaps();


	/** Removes the UV maps or UV map particle reference that contain the specified particle index.
	 * @param particleIndex A particle index to be matched.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh * RemoveMatchingUVMaps(int particleIndex);



	
	//Angle Constraint Operations

	/** Returns the count of the angle constraints. */
	int GetAngleConstraintCount(){
		return angleConstraints.size();
	}
	/** Returns the angle constraint at the specified index.
	 * @param index The index of the angle constraint to be get.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QAngleConstraint *GetAngleConstraintAt(int index){
		return angleConstraints[index];
	}
	/** Adds an angle constraint to mesh 
	 * @param angleConstraint An angle constraint to be removed.
	 * @return QMesh* A pointer to mesh itself.
	*/
	QMesh *AddAngleConstraint( QAngleConstraint * angleConstraint ){
		angleConstraints.push_back(angleConstraint);
		return this;
	}

	/** Removes an angle constraint to the mesh at the specified index.
	 * @param index The index of the angle constraint to be removed.
	 * @return QMesh* A pointer to mesh itself.
	 **/

	QMesh *RemoveAngleConstraintAt( int index ){
		angleConstraints.erase(angleConstraints.begin()+index );
		return this;
	}
	/** Returns the index of the specified angle constraint.
	 * @param angleConstraint An angle constraint in the mesh.
	 * @return If the angle constraint is found, it returns the index value, otherwise it returns -1.
	 */
	int GetAngleConstraintIndex(QAngleConstraint *angleConstraint){
		for(int i=0;i<angleConstraints.size();i++)
			if(angleConstraints[i]==angleConstraint)
				return i;
		return -1;
	}

	/** Removes an angle constraint to the mesh
	 * @param angleConstraint An angle constraint to be removed.
	 * @return QMesh* A pointer to mesh itself.
	 */
	QMesh *RemoveAngleConstraint( QAngleConstraint *angleConstraint ){
		int index=GetAngleConstraintIndex(angleConstraint);
		if(index!=-1){
			RemoveAngleConstraintAt(index);
		}
		return this;
	}

	/** Removes the angle constraint that contain the specified particle.
	 * @param particle A particle to be matched.
	 * @return QMesh* A pointer to mesh itself.
	 */

	QMesh *RemoveMatchingAngleConstraints(QParticle *particle){
		int i=0;
		while(i<angleConstraints.size()){
			QAngleConstraint *constraint=angleConstraints[i];
			if(constraint->GetParticleA()==particle || constraint->GetParticleB()==particle || constraint->GetParticleC()==particle){
				RemoveAngleConstraintAt(i);
			}else{
				++i;
			}
		}

		return this;
	}

	



	//Static Methods
	/**
	 * @brief Creates a mesh with a circle shape.
	 * @param radius The radius of the circle
	 * @param centerPosition The center position of the circle in the mesh.
	 * @return QMesh* A pointer to the created mesh.
	 */
	static QMesh * CreateWithCircle(float radius,QVector centerPosition=QVector::Zero());
	/**
	 * @brief Creates a mesh with a convex polygon shape.
	 * @param radius The radius of the regular polygon
	 * @param sideCount The number of sides of the polygon
	 * @param centerPosition The center position of the polygon in the mesh.
	 * @param polarGrid Applies a polar grid to the polygon with the specified layer count.(It's unnecessary for rigid bodies) 
	 * @param enableSprings Allow springs to the polygon.(It's unnecessary for rigid bodies)
	 * @param enablePolygons Allow polygon colliders to the mesh. Set to false for PBD type soft body needs.
	 * @param particleRadius The radius of the particles
	 * @return QMesh* A pointer to the created mesh.
	 */
	static QMesh * CreateWithPolygon(float radius,int sideCount,QVector centerPosition=QVector::Zero(),int polarGrid=-1,bool enableSprings=true, bool enablePolygons=true,float particleRadius=0.5f);

	/**
	 * @brief Creates a mesh with a rectangle shape
	 * @param size The size of the rectangle
	 * @param centerPosition The center position of the rectangle in the mesh. 
	 * @param grid The grid property of the rectangle. Applies a grid to the rectangle.(It's unnecessary for rigid bodies)
	 * @param enableSprings Allow springs to the rectangle.(It's unnecessary for rigid bodies)
	 * @param enablePolygons Allows polygon colliders to the mesh. Set to false for PBD type soft body needs.  
	 * @param particleRadius The radius of the particles.
	 * @return QMesh* A pointer to the created mesh.
	 */
	static QMesh * CreateWithRect(QVector size,QVector centerPosition=QVector::Zero(),QVector grid=QVector::Zero(),bool enableSprings=true, bool enablePolygons=true,float particleRadius=0.5f);

	/** Creates a mesh with a specified mesh data.
	 * @param data Mesh data. 
	 * @param enableSprings Allow springs to the mesh.(It's unnecessary for rigid bodies)
	 * @param enablePolygons Allow polygon colliders to the mesh. Set to false for PBD type soft body needs.  
	 * @return QMesh* A pointer to the created mesh.
	 */
	static QMesh * CreateWithMeshData(QMesh::MeshData &data,bool enableSprings=true, bool enablePolygons=true);

	/**Returns mesh data list from a json based *.qmesh file.You can use the returned mesh data of the collection with the QWorld::CreateWithMeshData method.
	 * @param filePath The filePath to load
	 * @return vector<QMesh::MeshData> A Mesh data list. 
	 */
	static vector<QMesh::MeshData> GetMeshDatasFromFile(string filePath);


	/**Returns mesh data list from a json based data.You can use the returned mesh data of the collection with the QWorld::CreateWithMeshData method. 
	 * @param jsonBasedData The json data to parse.
	 * @return vector<QMesh::MeshData> A Mesh data list. 
	 */
	static vector<QMesh::MeshData> GetMeshDatasFromJsonData(std::string &jsonBasedData);





	/**
	 * @brief Returns the area of a polygon
	 * @param polygonPoints A collection of particle pointers of the polygon
	 * @param withLocalPositions Defines whether the calculations will be done with local positions. 
	 */
	static float GetPolygonArea(vector<QParticle*> &polygonPoints,bool withLocalPositions=false);


	/**
	 * @brief Checks collision behaviors between two bodies. If the specified body pair is the same as the specified collision behavior pair, returns true. 
	 * @param meshA A mesh to check
	 * @param meshB Another mesh to check
	 * @param firstBehavior A behavior to check
	 * @param secondBehavior Another behavior to check. 
	 */
	static bool CheckCollisionBehaviors(QMesh *meshA,QMesh * meshB,CollisionBehaviors firstBehavior,CollisionBehaviors secondBehavior );
	/**
	 * @brief Generates rectangle mesh data with specified properties.
	 * @param size Size of the rectangle.
	 * @param centerPosition The center position of the rectangle. 
	 * @param grid The grid property of the rectangle. Applies a grid to the rectangle.(It's unnecessary for rigid bodies) 
	 * @param particleRadius The radius of the particles.
	 * @return MeshData The data needed to create a mesh.
	 */
	static MeshData GenerateRectangleMeshData(QVector size,QVector centerPosition=QVector::Zero(),QVector grid=QVector::Zero(),float particleRadius=0.5f);
	/**
	 * @brief Generates a regular polygon mesh data with specified properties.
	 * @param radius The radius of the regular polygon.
	 * @param sideCount The number of sides of the polygon. 
	 * @param centerPosition The center position of the polygon.
	 * @param polarGrid Applies a polar grid to the polygon with the specified layer count.(It's unnecessary for rigid bodies)
	 * @param particleRadius The radius of the particles.
	 * @return MeshData The data needed to create a mesh.
	 */
	static MeshData GeneratePolygonMeshData(float radius, int sideCount, QVector centerPosition=QVector::Zero(),int polarGrid=-1,float particleRadius=0.5f);

	/**Calculates the average position and rotation values of the specified particles. 
	 * @param particleCollection A Particles collection
	 * @return Returns a position-rotation pair.
	 */
	static pair<QVector, float> GetAveragePositionAndRotation(vector<QParticle*> particleCollection);

	/** Returns the non-deformed particle positions based on the target position and rotation. 
	 * This method is also used for shape matching operations.
	 * @param particleCollection  A collection of particles
	 * @param targetPosition  Target center position to transformation.
	 * @param targetRotation  Target center rotation to transformation.
	 * @return Returns A list of positions.
	*/
	static vector<QVector> GetMatchingParticlePositions(vector<QParticle*> particleCollection,QVector targetPosition, float targetRotation);


	/**
	 * By default, objects included in the physics engine are deleted by the destructors of the objects they belong to. When this flag is enabled, it indicates that this object should never be deleted by this engine. It is disabled by default, and it is recommended to keep it disabled. However, it can be used if needed for advanced purposes and integrations.
	 */
	bool manualDeletion=false;

	friend class QSoftBody;


	
};

#endif // QMESH_H
