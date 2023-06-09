#include "qworld.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>



using namespace std;
QWorld::QWorld(){



}


QWorld::~QWorld(){
	ClearWorld();
}


void QWorld::ClearGizmos(){
	for(auto gizmo:gizmos){
		delete gizmo;
	}
	gizmos.clear();
}

// ## WORLD STEP

void QWorld::Update(){
	ClearGizmos();


	for(auto body:bodies){
		body->Update();
	}

	//Constraints
	debugAABBTestCount=0;
	debugCollisionTestCount=0;
	//cout<<broadPhase.collisionGroups.size()<<endl;

	sort(bodies.begin(),bodies.end(),SortBodies);



	for(unsigned int n=0;n<iteration;++n){

		UpdateConstraints();
		manifolds.clear();
		if(enableBroadphase){
//			for(unsigned int i=0;i<bodies.size();i++){
//				bodies[i]->UpdateAABB();
//				broadPhase.Add(bodies[i]);
//			}
//			auto collisionPairs=broadPhase.GetPairs();
//			for(auto pair=collisionPairs->begin();pair!=collisionPairs->end();pair++){

//				QBody *bodyA=pair->first;
//				QBody *bodyB=pair->second;
//				Collide(bodyA,bodyB);
//			}




			int bodiesSize=bodies.size();
			for(unsigned int i=0;i<bodiesSize;++i){
				QBody* body=bodies[i];

				bool seperated=false;
				for(unsigned int q=i+1;q<bodiesSize;q++){
					QBody * otherBody=bodies[q];
					if( QBody::CanCollide(body,otherBody)==false){
						continue;
					}


					if(body->GetAABB().GetMax().x >= otherBody->GetAABB().GetMin().x){
						if( body->GetAABB().GetMin().y <= otherBody->GetAABB().GetMax().y &&
							body->GetAABB().GetMax().y >= otherBody->GetAABB().GetMin().y) {
							debugAABBTestCount+=1;
							vector<QCollision::Contact> contacts=GetCollisions(body,otherBody);
							if(contacts.size()>0){
								QManifold manifold(body,otherBody);
								manifold.contacts=contacts;
								manifolds.push_back(manifold);
							}


						}

					}else{
						break;
					}
				}

			}


		}else{
			//Brute Force Collision

			for (unsigned int ia=0;ia<bodies.size();++ia){
				QBody * bodyA=bodies[ia];
				for (unsigned int ib=ia+1;ib<bodies.size();++ib){
					QBody * bodyB=bodies[ib];
					debugAABBTestCount+=1;

					if(QBody::CanCollide(bodyA,bodyB)==false){
						continue;
					}

					vector<QCollision::Contact> contacts=GetCollisions(bodyA,bodyB);
					if(contacts.size()>0){
						QManifold manifold(bodyA,bodyB);
						manifold.contacts=contacts;
						manifolds.push_back(manifold);

					}

				}
			}
		}




		for(auto &manifold:manifolds){
			manifold.Solve();
		}
		for(auto &manifold:manifolds){
			manifold.SolveFrictionAndVelocities();
		}

		//Self particle collisions of soft bodies
		for(auto body:bodies){
			if(body->simulationModel!=QBody::SimulationModels::RIGID_BODY){
				QSoftBody *sBody=static_cast<QSoftBody*>(body);
				if(sBody!=nullptr){
					if(sBody->GetSelfParticleCollision()==true){
						for(int j=0;j<sBody->GetMeshCount();j++){
							QMesh *mesh=sBody->GetMeshAt(j);
							vector<QCollision::Contact> contacts;
							QCollision::CircleAndCircle(mesh->particles,mesh->particles,contacts);
							if(contacts.size()>0){
								QManifold manifold(sBody,sBody);
								manifold.contacts=contacts;
								manifold.Solve();
							}
						}
					}
				}
			}
		}





		for(auto body:bodies){
			body->UpdateAABB();
		}

	}



	for(auto raycast:raycasts){
		raycast->UpdateContacts();
	}


	//cout<<"Total Broad Phase Test Count: "<<debugAABBTestCount<<endl;
	//cout<<"Total Narrow Test Count: "<<debugCollisionTestCount<<endl;


	//Generating Islands and Sleeping Operations
	if(enableSleeping){
		sleepingIslands.clear();
		GenerateIslands(bodies,sleepingIslands);
		float oneDegree2Radian=M_PI/180;
		for(int i=0;i<sleepingIslands.size();i++){
			vector<QBody*> island=sleepingIslands[i];
			float velX=0.0f;
			float velY=0.0f;
			float angularVel=0.0f;
			bool islandNeedsAwake=false;
			for(auto body:island){

				velX=body->GetPosition().x-body->GetPreviousPosition().x;
				velY=body->GetPosition().y-body->GetPreviousPosition().y;
				angularVel=body->GetRotation()-body->GetPreviousRotation();

				if(velX>0.05f || velY>0.05f || angularVel>oneDegree2Radian ){
					islandNeedsAwake=true;
					break;
				}
//				cout<<"velX:"<<velX;
//				cout<<" velY:"<<velY;
//				cout<<" angularVel:"<<angularVel<<endl;


			}
			if(islandNeedsAwake==false){
				for(auto body:island){
					body->fixedVelocityTick+=1;
					body->fixedAngularTick+=1;

				}
			}else{
				for(auto body:island){
					body->fixedVelocityTick=0;
					body->fixedAngularTick=0;
					body->isSleeping=false;
				}
			}


		}

	}

}



// ## WORLD GENERAL METHODS

QWorld * QWorld::AddBody(QBody *body){
	bodies.push_back(body);
	body->world=this;
	return this;
}

QWorld * QWorld::AddBodyGroup(const vector<QBody*> &bodyGroup){
	for(int i=0;i<bodyGroup.size();i++){
		bodies.push_back(bodyGroup[i]);

	}
	return this;
}

QWorld *QWorld::RemoveBody(QBody *body)
{
	int index=GetBodyIndex(body);
	if(index!=-1){
		RemoveBodyAt(index);
	}
	return this;
}

QWorld *QWorld::RemoveBodyAt(int index)
{
	if(index>=0 && index<bodies.size()){
		QBody *body=bodies[index];
		//Remove collision exceptions if there is body
		RemoveMatchingCollisionException(body);
		//Remove joints if there is body
		RemoveMatchingJoints(body);
		//Remove springs if there is body
		RemoveMatchingSprings(body);

		bodies.erase(bodies.begin()+index);
	}

	return this;
}

vector<QBody *> QWorld::GetBodiesHitByPoint(QVector point, int maxBodyCount, bool onlyRigidBodies, int layersBit)
{
	vector<QBody*> res;
	res.reserve(maxBodyCount);


	QAABB pointAABB(point,point);
	for(int i=0;i<bodies.size();i++){
		auto body=bodies[i];
		if(onlyRigidBodies==true && body->simulationModel!=QBody::RIGID_BODY){
			continue;
		}
		if(enableBroadphase==true && body->GetPosition().x<point.x && body->GetPosition().x>point.x){
			continue;
		}

		if(body->GetAABB().isCollidingWith(pointAABB)==false){
			continue;
		}
		if(layersBit!=-1){
			if(body->GetOverlapWithLayersBit(layersBit)==false){
				continue;
			}
		}

		for(auto mesh:*body->GetMeshes()){
			if(mesh->collisionBehavior==QMesh::CIRCLES || mesh->collisionBehavior==QMesh::POLYLINE){
				for(int i=0;i<mesh->GetParticleCount();i++){
					QParticle *p=mesh->GetParticle(i);
					QVector diff=point-p->GetGlobalPosition();
					if(diff.Length()<p->GetRadius()){
						res.push_back(body);
						break;
					}

				}
			}
			if(mesh->collisionBehavior==QMesh::POLYGONS || mesh->collisionBehavior==QMesh::POLYLINE){
				for(int i=0;i<mesh->GetClosedPolygonCount();i++){
					vector<QParticle*> polygon=mesh->GetClosedPolygon(i);
					if( QCollision::PointInPolygon(point,polygon) ){
						res.push_back(body);
						break;
					}
				}

			}

		}
		if(res.size()==maxBodyCount)break;
	}

	return res;
}

vector<QParticle *> QWorld::GetParticlesCloseToPoint(QVector point, float distance, int maxParticleCount, bool exceptRigidBodies, int layerMask)
{
	vector<QParticle*> res;
	res.reserve(maxParticleCount);


	QAABB pointAABB(point,point);
	for(int i=0;i<bodies.size();i++){
		auto body=bodies[i];
		if(exceptRigidBodies==true && body->simulationModel==QBody::RIGID_BODY){
			continue;
		}
		if(enableBroadphase==true && body->GetPosition().x<point.x && body->GetPosition().x>point.x){
			continue;
		}

		QAABB bodyFattedAABB=body->GetAABB().Fatted(distance);
		if(bodyFattedAABB.isCollidingWith(pointAABB)==false){
			continue;
		}
		for(auto mesh:*body->GetMeshes()){
			for(int i=0;i<mesh->GetParticleCount();i++){
				QParticle *p=mesh->GetParticle(i);
				QVector diff=point-p->GetGlobalPosition();
				if(diff.Length()<distance){
					res.push_back(p);
					if(res.size()==maxParticleCount)break;
				}

			}


		}
		if(res.size()==maxParticleCount)break;
	}

	return res;
}







void QWorld::ClearBodies(){
	for(int i=0;i<bodies.size();i++){
		delete bodies[i];
	}
}
QWorld* QWorld::ClearJoints(){
	for(int i=0;i<joints.size();i++){
		delete joints[i];
	}
	return this;
}

QWorld* QWorld::ClearSprings()
{
	for(int i=0;i<springs.size();i++){
		delete springs[i];
	}
	return this;
}

QWorld* QWorld::ClearRaycasts()
{
	for(int i=0;i<raycasts.size();i++){
		delete raycasts[i];
	}
	return this;
}

QWorld* QWorld::ClearWorld(){
	ClearBodies();
	ClearJoints();
	ClearSprings();
	ClearRaycasts();
	ClearGizmos();
	return this;
}

QWorld *QWorld::AddJoint(QJoint *joint)
{
	joints.push_back(joint);
	return this;
}

QWorld *QWorld::RemoveJoint(QJoint *joint)
{
	auto it=find(joints.begin(),joints.end(),joint);
	if(it!=joints.end()){
		joints.erase(it);
	}

	return this;

}

QWorld *QWorld::RemoveJointAt(int index)
{
	joints.erase(joints.begin()+index);
	return this;
}

QWorld *QWorld::RemoveMatchingJoints(QBody *body)
{
	int i=0;
	while(i<joints.size()){
		QJoint *joint=joints[i];
		if(joint->GetBodyA()==body || joint->GetBodyB()==body){
			joints.erase(joints.begin()+i);
		}else{
			++i;
		}
	}
	return this;
}

QWorld *QWorld::AddSpring(QSpring *spring)
{
	springs.push_back(spring);
	return this;
}

QWorld *QWorld::RemoveSpring(QSpring *spring)
{
	auto it=find(springs.begin(),springs.end(),spring);
	if(it!=springs.end()){
		springs.erase(it);
	}
	return this;

}

QWorld *QWorld::RemoveSpringAt(int index)
{
	springs.erase(springs.begin()+index);
	return this;

}

QWorld *QWorld::RemoveMatchingSprings(QBody *body)
{
	int i=0;
	while(i<springs.size()){
		QSpring *spring=springs[i];
		bool matched=false;
		if(spring->GetParticleA()!=nullptr){
			if(spring->GetParticleA()->GetOwnerMesh()!=nullptr){
				if(spring->GetParticleA()->GetOwnerMesh()->ownerBody==body){
					matched=true;
				}
			}
		}
		if(spring->GetParticleB()!=nullptr){
			if(spring->GetParticleB()->GetOwnerMesh()!=nullptr){
				if(spring->GetParticleB()->GetOwnerMesh()->ownerBody==body){
					matched=true;
				}
			}
		}
		if(matched==true){
			springs.erase(springs.begin()+i);
		}else{
			++i;
		}
	}
	return this;
}

QWorld *QWorld::RemoveMatchingSprings(QParticle *particle)
{
	int i=0;
	while(i<springs.size()){
		QSpring *spring=springs[i];
		bool matched=false;
		if(spring->GetParticleA()==particle || spring->GetParticleB()==particle){
			springs.erase(springs.begin()+i);
		}else{
			++i;
		}
	}
	return this;
}

QWorld *QWorld::AddRaycast(QRaycast *raycast)
{
	raycasts.push_back(raycast);
	raycast->world=this;

	return this;
}

QWorld *QWorld::RemoveRaycast(QRaycast *raycast)
{
	auto it=find(raycasts.begin(),raycasts.end(),raycast);
	if(it!=raycasts.end()){
		raycasts.erase(it);
	}
	return this;
}

QWorld *QWorld::RemoveRaycastAt(int index)
{
	raycasts.erase(raycasts.begin()+index);
	return this;
}

QWorld *QWorld::AddCollisionException(QBody *bodyA, QBody *bodyB)
{
	pair<QBody*,QBody*> npair;
	if(bodyA<bodyB){
		npair.first=bodyA;
		npair.second=bodyB;
	}else{
		npair.first=bodyB;
		npair.second=bodyA;
	}
	collisionExceptions.insert(npair);

	return this;
}

QWorld *QWorld::RemoveCollisionException(QBody *bodyA, QBody *bodyB)
{
	if(collisionExceptions.empty()==true)return this;
	pair<QBody*,QBody*> npair;
	if(bodyA<bodyB){
		npair.first=bodyA;
		npair.second=bodyB;
	}else{
		npair.first=bodyB;
		npair.second=bodyA;
	}
	collisionExceptions.erase(npair);

	return this;
}

QWorld *QWorld::RemoveMatchingCollisionException(QBody *body)
{
	if(collisionExceptions.empty()==true)return this;
	auto it=collisionExceptions.begin();
	while(it!=collisionExceptions.end()){
		if(it->first==body || it->second==body){
			it=collisionExceptions.erase(it);
		}else{
			++it;
		}
	}
	return this;
}

bool QWorld::CheckCollisionException(QBody *bodyA, QBody *bodyB)
{
	if(collisionExceptions.empty()==true)return false;
	pair<QBody*,QBody*> npair;
	if(bodyA<bodyB){
		npair.first=bodyA;
		npair.second=bodyB;
	}else{
		npair.first=bodyB;
		npair.second=bodyA;
	}
	if(collisionExceptions.find(npair)==collisionExceptions.end()){
		return false;
	}
	return true;
}


// ## WORLD CONSTRAINT METHODS


//Collision Constraints and Response Between Bodies
vector<QCollision::Contact> QWorld::GetCollisions(QBody *bodyA, QBody *bodyB){
	vector<QCollision::Contact> contactList{};

	if(bodyA->GetIsSleeping() && bodyB->GetIsSleeping())
		return contactList;



	vector<QMesh*>* meshesA=bodyA->GetMeshes();
	vector<QMesh*>* meshesB=bodyB->GetMeshes();




	for(int sa=0;sa<meshesA->size();++sa){
		QMesh *meshA=meshesA->at(sa);
		for(int sb=0;sb<meshesB->size();++sb){
			QMesh *meshB=meshesB->at(sb);

			bodyA->GetWorld()->debugCollisionTestCount+=1;
			if(QMesh::CheckCollisionBehaviors(meshA,meshB,QMesh::POLYGONS, QMesh::POLYGONS )){
				for(int a=0;a<meshA->closedPolygons.size();a++){
					for(int b=0;b<meshB->closedPolygons.size();b++){
						QCollision::PolygonAndPolygon(meshA->closedPolygons[a],meshB->closedPolygons[b],contactList,bodyA->GetWorld());
					}
				}

			}else if(QMesh::CheckCollisionBehaviors(meshA,meshB,QMesh::CIRCLES, QMesh::POLYGONS )){
				QMesh *circleMesh=meshA->collisionBehavior==QMesh::CIRCLES ? meshA:meshB;
				QMesh *polygonMesh=&circleMesh->particles==&meshA->particles ? meshB :meshA;
				for(int j=0;j<polygonMesh->GetClosedPolygonCount();j++){
					QCollision::CircleAndPolygon(circleMesh->particles,polygonMesh->GetClosedPolygon(j),contactList);
				}

			}else if(QMesh::CheckCollisionBehaviors(meshA,meshB,QMesh::CIRCLES, QMesh::CIRCLES )){
				QCollision::CircleAndCircle(meshA->particles,meshB->particles,contactList);

			}else if(QMesh::CheckCollisionBehaviors(meshA,meshB,QMesh::POLYLINE, QMesh::POLYGONS )){
				QMesh *polylineMesh=meshA->collisionBehavior==QMesh::POLYLINE ? meshA:meshB;
				QMesh *polygonMesh=meshA->collisionBehavior==QMesh::POLYGONS ? meshA:meshB;
				for(int a=0;a<polylineMesh->closedPolygons.size();a++){
					for(int b=0;b<polygonMesh->closedPolygons.size();b++){
						QCollision::CircleAndPolygon(polylineMesh->GetClosedPolygon(a),polygonMesh->GetClosedPolygon(b),contactList);
						QCollision::PolylineAndPolygon(polylineMesh->GetClosedPolygon(a),polygonMesh->GetClosedPolygon(b),contactList);
					}
				}

			}else if(QMesh::CheckCollisionBehaviors(meshA,meshB,QMesh::POLYLINE, QMesh::POLYLINE )){
				for(int a=0;a<meshA->closedPolygons.size();a++){
					for(int b=0;b<meshB->closedPolygons.size();b++){
						QCollision::PolylineAndPolygon(meshA->GetClosedPolygon(a),meshB->GetClosedPolygon(b),contactList);
						QCollision::PolylineAndPolygon(meshB->GetClosedPolygon(b),meshA->GetClosedPolygon(a),contactList);
					}
				}


			}else if(QMesh::CheckCollisionBehaviors(meshA,meshB,QMesh::POLYLINE, QMesh::CIRCLES )){
				QMesh *circleMesh=meshA->collisionBehavior==QMesh::CIRCLES ? meshA:meshB;
				QMesh *polylineMesh=meshA->collisionBehavior==QMesh::POLYLINE ? meshA:meshB;
				for(int j=0;j<polylineMesh->closedPolygons.size();j++){
					QCollision::PolylineAndCircle(polylineMesh->GetClosedPolygon(j),circleMesh->particles,contactList);
				}

			}


		}

	}



	return contactList;






}

//Collision Islands
void QWorld::CreateIslands(QBody *body, vector<QBody*> &bodyList, vector<QBody *> &island)
{
	if(body->visited)return;
	if(body->GetMode()==QBody::Modes::STATIC)return;

	//We visited this body
	body->visited=true;

	island.push_back(body);

	//Search other AABB collided objects
	for(int i=0;i<bodyList.size();i++){
		QBody* otherBody=bodies[i];
		QAABB otherAABB=otherBody->GetAABB();
		if (body != otherBody && body->GetAABB().isCollidingWith(otherAABB) )
		{
			// If there is a collision, visit to otherBody
			CreateIslands(otherBody,bodyList, island);
		}
	}

}


void QWorld::GenerateIslands(vector<QBody *> &bodyList, vector<vector<QBody *>> &islandList)
{
	// define all bodies as not visited
	for (int i = 0; i < bodyList.size(); i++)
	{
		bodyList[i]->visited = false;
	}



	// Call DFS method to all bodies
	for (int i = 0; i < bodyList.size(); i++)
	{
		QBody* body = bodyList[i];
		if(body->GetMode()==QBody::Modes::STATIC)
			continue;
		if (!body->visited)
		{
			// If we don't visited this body, create a new island
			vector<QBody*> island = vector<QBody*>();
			CreateIslands(body,bodyList, island);
			islandList.push_back(island);
		}
	}
}

bool QWorld::SortBodies(const QBody *bodyA, const QBody *bodyB)
{
	if(bodyA->GetAABB().GetMin().x==bodyB->GetAABB().GetMin().x){
		return bodyA->GetAABB().GetMax().y>bodyB->GetAABB().GetMax().y;
	}
	return bodyA->GetAABB().GetMin().x<bodyB->GetAABB().GetMin().x;
}

bool QWorld::SortBodiesVertical(const QBody *bodyA, const QBody *bodyB)
{

	return bodyA->GetAABB().GetMin().y>bodyB->GetAABB().GetMin().y;
}

 void QWorld::GetCollisionPairs(vector<QBody *> &bodyList, vector<pair<QBody *, QBody *> > *resList)
{
	//Using Sweep and Purne Algorithm
	sort(bodies.begin(),bodies.end(),SortBodies);

	int bodiesSize=bodies.size();
	for(unsigned int i=0;i<bodiesSize;++i){
		QBody* body=bodies[i];

		bool seperated=false;
		for(unsigned int q=i+1;q<bodiesSize;q++){
			QBody * otherBody=bodies[q];


			if(body->GetAABB().GetMin().x <= otherBody->GetAABB().GetMax().x){
				if(body->GetAABB().GetMax().x >= otherBody->GetAABB().GetMin().x &&
					body->GetAABB().GetMin().y <= otherBody->GetAABB().GetMax().y &&
					body->GetAABB().GetMax().y >= otherBody->GetAABB().GetMin().y) {
					debugAABBTestCount+=1;
					resList->push_back(make_pair(body,otherBody));
				}

			}else{
				seperated=true;
				break;
			}
		}
		if(seperated)
			continue;
	}

 }

 void QWorld::UpdateConstraints()
 {
	 //Soft Body Constraints
	 for(auto body:bodies){
		 if(body->GetMode()!=QBody::STATIC && body->GetSimulationModel()!=QBody::RIGID_BODY){
			 QSoftBody *sBody=static_cast<QSoftBody*>(body);
			 for(int i=0;i<sBody->GetMeshCount();i++){
				 QMesh * mesh=sBody->GetMeshAt(i);
				 for(auto spring:mesh->springs){
					 spring->Update(sBody->GetRigidity(),sBody->GetPassivationOfInternalSprings());
				 }
			 }
		 }
	 }
	 for(auto spring:springs){
		 spring->Update(spring->GetRigidity(),false);
	 }
	 //Joint Constraints
	 for(auto joint:joints){
		 joint->Update();
	 }
 }



