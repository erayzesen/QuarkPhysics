
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

#include "qmesh.h"
#include "qbody.h"
#include "qvector.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "qcollision.h"
#include "qworld.h"
#include "polypartition/polypartition.h"
#include <list>


QMesh::QMesh(){
}

QMesh::~QMesh()
{
	for(int i=0;i<particles.size();i++){
		if (particles[i]!=nullptr){
			if(particles[i]->manualDeletion==false ){
				delete particles[i];
				particles[i]=nullptr;
			}
		}
	}
	particles.clear();

	for(int i=0;i<springs.size();i++){
		if(springs[i]!=nullptr){
			if(springs[i]->manualDeletion==false){
				delete springs[i];
				springs[i]=nullptr;
			}
		}
	}
	springs.clear();

	for(int i=0;i<angleConstraints.size();i++){
		if(angleConstraints[i]!=nullptr){
			if(angleConstraints[i]->manualDeletion==false){
				delete angleConstraints[i];
				angleConstraints[i]=nullptr;
			}
		}
	}
	angleConstraints.clear();

}

void QMesh::UpdateCollisionBehavior()
{
	if(ownerBody==nullptr)
		return;
	

	if(GetPolygonParticleCount()>0 && disablePolygonForCollisions==false){
		if(ownerBody->simulationModel==QBody::SimulationModels::RIGID_BODY){
			collisionBehavior=CollisionBehaviors::POLYGONS;
		}else{
			collisionBehavior=CollisionBehaviors::POLYLINE;
		}

	}else{
		collisionBehavior=CollisionBehaviors::CIRCLES;
	}
}

//Particles

QMesh *QMesh::AddParticle(QParticle *particle){
	particles.push_back(particle);
	particles.back()->SetOwnerMesh(this);
	if(ownerBody!=nullptr){
		if (ownerBody->mode==QBody::Modes::STATIC)
			ownerBody->UpdateMeshTransforms();
		ownerBody->inertiaNeedsUpdate=true;
		ownerBody->circumferenceNeedsUpdate=true;
	}
	collisionBehaviorNeedsUpdate=true;
	return this;
}

QMesh *QMesh::RemoveParticleAt(int index){
	QParticle *particle=particles[index];
	RemoveParticleFromPolygon(particle);
	RemoveMatchingSprings(particle);
	RemoveMatchingUVMaps(index);
	RemoveMatchingAngleConstraints(particle);
	particles.erase(particles.begin()+index);
	if(ownerBody!=nullptr){
		if (ownerBody->mode==QBody::Modes::STATIC)
			ownerBody->UpdateMeshTransforms();
		ownerBody->inertiaNeedsUpdate=true;
		ownerBody->circumferenceNeedsUpdate=true;
	}
	collisionBehaviorNeedsUpdate=true;
	polygonBisectorsNeedsUpdate=true;
	return this;
}

QMesh *QMesh::RemoveParticle(QParticle *particle){
	int index=GetParticleIndex(particle);
	if(index!=-1)
		RemoveParticleAt(index);
	return this;
}

int QMesh::GetParticleCount(){
	return particles.size();
}

QParticle *QMesh::GetParticleAt(int index){
	return particles[index];
}

//Polygons

QVector QMesh::GetAveragePosition()
{
    if(particles.size()==1)
		return particles[0]->GetGlobalPosition();
	//Finding Actual Position
	QVector averagePosition=QVector::Zero();
	
	for(int i=0;i<particles.size();i++){
		QParticle *particle=particles[i];
		averagePosition+=particle->GetGlobalPosition();
	}  
	averagePosition/=particles.size();

	return averagePosition;
}

float QMesh::GetAverageRotation()
{
	return GetAveragePositionAndRotation(particles).second;
}

QMesh *QMesh::SetPolygon(vector<QParticle *> polygonParticles)
{
	polygon=polygonParticles;

	subConvexPolygonsNeedsUpdate=true;

	collisionBehaviorNeedsUpdate=true;

	polygonBisectorsNeedsUpdate=true;

    return this;
}

QMesh *QMesh::AddParticleToPolygon(QParticle *particle, int position)
{
	if (position==-1) {
		polygon.push_back(particle);
	}else{
		polygon.insert(polygon.begin()+position,particle);
	}
	subConvexPolygonsNeedsUpdate=true;

	collisionBehaviorNeedsUpdate=true;

	polygonBisectorsNeedsUpdate=true;

    return this;
}

QMesh *QMesh::RemoveParticleFromPolygon(QParticle *particle)
{
	int index=-1;
	for (int i=0;i<polygon.size();i++) {
		if (polygon[i]==particle)
			index=i;
			break;
	}
	if (index!=-1){
		RemoveParticleFromPolygonAt(index);
	}
    return this;
}

QMesh *QMesh::RemoveParticleFromPolygonAt(int index)
{
	polygon.erase(polygon.begin()+index);
	subConvexPolygonsNeedsUpdate=true;
	collisionBehaviorNeedsUpdate=true;
	polygonBisectorsNeedsUpdate=true;
    return this;
}

QMesh *QMesh::RemovePolygon()
{
	polygon.clear();
	subConvexPolygons.clear();
	collisionBehaviorNeedsUpdate=true;
	polygonBisectorsNeedsUpdate=true;
    return this;
}

int QMesh::GetPolygonParticleCount()
{
    return polygon.size();
}

QParticle *QMesh::GetParticleFromPolygon(int index)
{
    return polygon[index];
}

void QMesh::UpdateSubConvexPolygons( bool majorUpdate)
{
	
	if(subConvexPolygons.size()==0){
		DecompositePolygon(polygon,subConvexPolygons);	
	}else{
		if(majorUpdate==true){
			subConvexPolygons.clear();
			if(CheckIsPolygonConcave(polygon)==true	){
				DecompositePolygon(polygon,subConvexPolygons);
			}else{
				subConvexPolygons.push_back( polygon );		
			}
		}else{
			bool subPolygonsAreConcave=false;
			for(auto poly:subConvexPolygons){
				if(CheckIsPolygonConcave(poly)==true ){
					subPolygonsAreConcave=true;
					break;
				}
			}
			if(subPolygonsAreConcave){
				subConvexPolygons.clear();
				DecompositePolygon(polygon,subConvexPolygons);	
			}
		}
	}
	
	
}

void QMesh::UpdatePolygonBisectors()
{
	if(polygonBisectorsNeedsUpdate==true){
		polygonBisectors=QMesh::GetBisectors(polygon);
		polygonBisectorsNeedsUpdate=false;
	}
	
}

vector<QVector> QMesh::GetBisectors(vector<QParticle *> polygonParticles)
{
	vector<QVector> res;
	for (size_t i=0;i<polygonParticles.size();i++ ){
		QParticle *pp=polygonParticles[ (i-1+polygonParticles.size() )%polygonParticles.size() ]; //previous particle
		QParticle *p=polygonParticles[i]; // particle
		QParticle *np=polygonParticles[ (i+1 )%polygonParticles.size() ]; //next particle

		QVector bisectorUnit=QVector::GeteBisectorUnitVector(pp->GetGlobalPosition(), p->GetGlobalPosition(), np->GetGlobalPosition(),true);

		float bisectorLen=1.0f;
		float bestDistance=QWorld::MAX_WORLD_SIZE;
		for(size_t n=0;n<polygonParticles.size();++n ){
			QParticle *s1=polygonParticles[n];
			QParticle *s2=polygonParticles[ (n+1)%polygonParticles.size() ];
			if(s1==pp || s1==np || s1==p){
				continue;
			}
			QVector bisectorIntersection=QCollision::LineIntersectionLine(p->GetGlobalPosition(),p->GetGlobalPosition()+bisectorUnit*QWorld::MAX_WORLD_SIZE,s1->GetGlobalPosition(),s2->GetGlobalPosition()  );
			if(bisectorIntersection!=QVector::NaN() ){
				float len=(p->GetGlobalPosition()-bisectorIntersection).Length();
				if(len<bestDistance){
					bisectorLen=len;
					bestDistance=len;
				}
			}
		}

		res.push_back(bisectorUnit*bisectorLen);


	}
    return res;
}

void QMesh::ApplyAngleConstraintsToPolygon()
{
	if(minAngleConstraintOfPolygon==0.0f){
		return;
	}

	//Intersection Test
	bool polygonIntersection=false;
	for(int i=0;i<polygon.size();++i ){
		int ni=(i+1)%polygon.size();
		QParticle* d1A=polygon[i];
		QParticle* d1B=polygon[ (i+1)%polygon.size() ];
		for(int n=i+1;n<polygon.size();++n ){
			if(n==i || n==ni || n==(i-1+polygon.size())%polygon.size() )continue;
			QParticle* d2A=polygon[n];
			QParticle* d2B=polygon[ (n+1)%polygon.size() ];

			QVector intersection=QCollision::LineIntersectionLine(d1A->GetGlobalPosition(),d1B->GetGlobalPosition(),d2A->GetGlobalPosition(),d2B->GetGlobalPosition() );
			
			if(intersection.isNaN()==false ){
				d1A->GetOwnerMesh()->GetOwnerBody()->GetWorld()->GetGizmos()->push_back(new QGizmoCircle(intersection,5.0) );
				polygonIntersection=true;
				break;
			}
		}

	}
	isPolygonSelfIntersected=polygonIntersection;
	if(polygonIntersection==true){
		//cout<<"there is line intersection in polygon"<<endl;
		pair<QVector,float> averagePosRot=QMesh::GetAveragePositionAndRotation(polygon);
		vector<QVector> matchingShape=QMesh::GetMatchingParticlePositions(polygon,averagePosRot.first,averagePosRot.second);
		for(int i=0;i<matchingShape.size();i++ ){
			if (polygon[i]->GetEnabled()==false )
				continue;
			QVector force=(matchingShape[i]-polygon[i]->GetGlobalPosition())*0.2;
			polygon[i]->ApplyForce(force );
			lastPolygonCornerAngles.clear();
			return;
		}
	}

	bool beginToSaveAngles=false;

	if (lastPolygonCornerAngles.size()!=polygon.size() ){
		lastPolygonCornerAngles.clear();
		for(size_t i=0;i<polygon.size();++i ){
			lastPolygonCornerAngles.push_back(0.0f);
		}
		beginToSaveAngles=true;
	}
	
	float minAngle=minAngleConstraintOfPolygon;
	float maxAngle=(M_PI*2.0)-minAngle;

	for (size_t i=0;i<polygon.size();++i ){
		size_t pi=(i-1+polygon.size())%polygon.size();
		size_t ni=(i+1)%polygon.size();

		QParticle *pp=polygon[pi];
		QParticle *p=polygon[i];
		QParticle *np=polygon[ni];

		QVector toPrev=pp->GetGlobalPosition()-p->GetGlobalPosition();
		QVector toNext=np->GetGlobalPosition()-p->GetGlobalPosition();

		float cosA=toNext.Dot(toPrev)/(toPrev.Length()*toNext.Length() );
		float sinA=toNext.Dot(toPrev.Perpendicular() )/(toPrev.Length()*toNext.Length() );

		float angleRad=atan2(sinA,cosA);



		if(angleRad<0){
			angleRad=(M_PI*2.0)-abs(angleRad);
		}


		
		if(beginToSaveAngles){
			lastPolygonCornerAngles[i]=angleRad;
			continue;
		}


		QVector d1=QVector::AngleToUnitVector(lastPolygonCornerAngles[i]);
		QVector d2=QVector::AngleToUnitVector(angleRad);
		float angleDifference=QVector::AngleBetweenTwoVectors(d2,d1);

		angleRad=lastPolygonCornerAngles[i]+angleDifference;

		


		if(angleRad>maxAngle){
			float diffAngle=maxAngle-angleRad;
			float angularForce=diffAngle*0.5f;

			
			if (pp->GetEnabled() )
				pp->SetGlobalPosition( p->GetGlobalPosition()+toPrev.Rotated(angularForce) );
			if (np->GetEnabled() )
				np->SetGlobalPosition( p->GetGlobalPosition()+toNext.Rotated(-angularForce) );

			
			
		}

		if(angleRad<minAngle){
			float diffAngle=minAngle-angleRad;
			float angularForce=diffAngle*0.5f;

			if (pp->GetEnabled() )
				pp->SetGlobalPosition( p->GetGlobalPosition()+toPrev.Rotated(angularForce) );
			if (np->GetEnabled() )
				np->SetGlobalPosition( p->GetGlobalPosition()+toNext.Rotated(-angularForce) );

			
		}

		lastPolygonCornerAngles[i]=angleRad;

		
	}

}

bool QMesh::CheckIsPolygonConcave(vector<QParticle *> polygonParticles)
{
	int polygonParticleCount=GetPolygonParticleCount();
	for (size_t i=0;i<polygonParticleCount;i++ ){
		QVector p1=GetParticleFromPolygon( (i-1+polygonParticleCount)%polygonParticleCount )->GetGlobalPosition();
		QVector p2=GetParticleFromPolygon( i )->GetGlobalPosition();
		QVector p3=GetParticleFromPolygon( (i+1)%polygonParticleCount )->GetGlobalPosition();
		if (CheckIsReflex(p1,p2,p3) )
			return true;
	}
    return false;
}

bool QMesh::CheckIsReflex(QVector pA, QVector pB, QVector pC)
{
	if ( (pB-pA).Dot( (pC-pA).Perpendicular() )<-1.0f )
		return true;
	return false;
}

bool QMesh::CheckIsReflex(int indexA, int indexB, int indexC, vector<QParticle *> polygonParticles)
{
    return CheckIsReflex( polygonParticles[indexA]->GetGlobalPosition(),polygonParticles[indexB]->GetGlobalPosition(),polygonParticles[indexC]->GetGlobalPosition() );
}

void QMesh::TriangulatePolygon(vector<QParticle *> &polygonParticles, vector<vector<int>> &triangles)
{
	vector<int> indexList;
	for (int i=0;i<polygonParticles.size();i++ ){
		indexList.push_back(i);
	}

	int maxIterationCount=indexList.size()*3;
	int iterationCount=0;

	
	while (indexList.size()>3 ){
		iterationCount+=1;
		if (iterationCount>maxIterationCount){
			break;
		}
		
		
		for (int i=0;i<indexList.size();i++ ){
			int pi=indexList[ (i-1+indexList.size() )%indexList.size() ]; //previous index
			int ci=indexList[i]; //current index
			int ni=indexList[ (i+1)%indexList.size() ]; // next index

			QVector pp=polygonParticles[pi]->GetPosition(); // previous particle position
			QVector cp=polygonParticles[ci]->GetPosition(); // current particle position
			QVector np=polygonParticles[ni]->GetPosition(); // next particle position
			

			//Checking whether the vertice is ear  
			if ( CheckIsReflex(pp,cp,np) )
				continue;

			QVector cp2npPerp=(np-cp).Perpendicular();
			QVector np2ppPerp=(pp-np).Perpendicular();
			QVector pp2cpPerp=(cp-pp).Perpendicular();
			
			// Checking other vertices are in the triangle 
			bool isThereAVertice=false;
			for (int n=0;n<indexList.size();n++ ){
				int ti=indexList[n]; // test index
				if (ti==pi || ti==ci || ti==ni)
					continue;
				QVector tp=polygonParticles[ti]->GetPosition(); // test particle position

				float d1=(tp-cp).Dot( cp2npPerp );
				float d2=(tp-np).Dot( np2ppPerp );
				float d3=(tp-pp).Dot( pp2cpPerp );

				if ( (d1>0 && d2>0 && d3>0 ) || (d1<0 && d2<0 && d3<0) ){
					isThereAVertice=true;
					break;
				}

			}

			//Adding a new triangle and removing ear from the polygon
			if (isThereAVertice==false){
				vector<int> triangle={pi,ci,ni};
				triangles.push_back ( triangle);
				indexList.erase(indexList.begin()+i);
				break;
			}

		}
	}

	//Adding the final three points as a triangle to the triangles
	vector<int> lastTriangle;
	for (int i=0;i<indexList.size();i++ ){
		lastTriangle.push_back(indexList[i] );
	}
	triangles.push_back(lastTriangle);


}

void QMesh::DecompositePolygon2(vector<QParticle *> &polygonParticles, vector<vector<QParticle *>> &polygons)
{
	
	vector<vector<int>> subPolygons;
	TriangulatePolygon(polygonParticles,subPolygons);

	
	
	int ia=0;

	//Per all subPolygons
	while (ia!=subPolygons.size()){
		// cout<<"started subpolygons loop ia is:"<<ia <<endl;
		auto polyA=subPolygons[ia];
		bool isDiagonal=false;
		//Check diagonals
		for (size_t na=0;na<polyA.size();na++ ){
			int pA1=polyA[na ];
			int pA2=polyA[ (na+1)%polyA.size() ];

			for (size_t ib=0;ib<subPolygons.size();ib++ ){
				if (ia==ib)
					continue;
				// cout<<"started subpolygons loop ib is:"<<ib <<endl;
				auto polyB=subPolygons[ib];
				for (size_t nb=0;nb<polyB.size();nb++ ){
					int pB1=polyB[nb];
					int pB2=polyB[ (nb+1)%polyB.size() ];
					if (pB2==pA1 && pB1==pA2){
						//Finded a diagonal polygon. checking reflex... 
						if (CheckIsReflex( polyA[ (na-1+polyA.size() )%polyA.size() ], pA1, polyB[(nb+2) % polyB.size() ],polygonParticles ) )
							continue;
						if ( CheckIsReflex(polyB[ (nb-1+polyB.size())%polyB.size() ], pA2, polyA[ (na+2)%polyA.size() ],polygonParticles )  )
							continue;
						//There is no exist reflexes. So the new polygon can be convex.
						int pIndex=(nb+1)%polyB.size();
						int pos=na+1;
						//Adding points to first polygon
						// cout<<"adding next polygon to prev polygon"<<endl;
						while (pIndex!=(nb-1+polyB.size() )%polyB.size() ){
							pIndex=(pIndex+1)%polyB.size();
							polyA.insert(polyA.begin()+pos,polyB[pIndex]);
							pos+=1;
						}
						subPolygons[ia]=polyA;
						// cout<<"erased polygon"<<endl;
						subPolygons.erase(subPolygons.begin()+ib );
						// cout<<"erased polygon finished"<<endl;
						isDiagonal=true;
						break;
					}
				}
				if (isDiagonal){
					// cout<<"exiting for polyB  loop"<<endl;
					break;
				}
			}
			if (isDiagonal){
				// cout<<"exiting for polyA loop"<<endl;
				break;	
			}
		}
		if (isDiagonal){
			// cout<<"continue subpolygons loop ia is:"<<ia <<endl;
			continue;
		}
		ia+=1;
	}
	//Converting particle index polygon collection  to QParticle polygon collection  
	for (size_t i=0;i<subPolygons.size();i++ ){
		vector<QParticle*> poly;
		for(size_t n=0;n<subPolygons[i].size();n++ ){
			int pIndex=subPolygons[i][n];
			poly.push_back(polygonParticles[pIndex] );
			
		}
		polygons.push_back(poly);
	}

	

}

void QMesh::DecompositePolygon(vector<QParticle *> &polygonParticles, vector<vector<QParticle *>> &polygons)
{
	vector<QVector> polygon;
	for (int i=0;i<polygonParticles.size();++i ){
		polygon.push_back(polygonParticles[i]->GetGlobalPosition() );
	}
	
	list<TPPLPoly> in_poly;
	list<TPPLPoly> out_poly;
	

	TPPLPoly inp;
	inp.Init(polygon.size());
	for (int i = 0; i < polygon.size(); i++) {
		TPPLPoint point;
		point.x=polygon[i].x;
		point.y=polygon[i].y;
		inp.GetPoint(i) = point;
		inp.GetPoint(i).id=i;
	}
	inp.SetOrientation(TPPL_ORIENTATION_CCW);
	in_poly.push_back(inp);
	TPPLPartition tpart;
	if (tpart.ConvexPartition_HM(&in_poly, &out_poly) == 0) { // Failed.
		cout<<"QuarkPhysics Error: Convex decomposing failed!"<<endl;
		return;
	}

	for (auto poly:out_poly){
		vector<QParticle*> qpoly;
		for(int i=0;i<poly.GetNumPoints();++i){
			int point_index=poly.GetPoint(i).id;
			qpoly.push_back(polygonParticles[point_index]);
			
		}
		polygons.push_back(qpoly);
	}
	

	
}

//Springs

QMesh *QMesh::AddSpring(QSpring *spring)
{
	springs.push_back(spring);
	if(spring->GetParticleA()!=nullptr || spring->GetParticleB()!=nullptr){
		spring->GetParticleA()->springConnectedParticles.insert(spring->GetParticleB() );
		spring->GetParticleB()->springConnectedParticles.insert(spring->GetParticleA() );
	}
	collisionBehaviorNeedsUpdate=true;
	return this;
}

QMesh *QMesh::RemoveSpring(QSpring *spring)
{
	int it=GetSpringIndex(spring);
	if(it!=-1)
		RemoveSpringAt(it);

	return this;
}

QMesh *QMesh::RemoveSpringAt(int index)
{
	QSpring *spring=springs[index];

	spring->GetParticleA()->springConnectedParticles.erase(spring->GetParticleB() );
	spring->GetParticleB()->springConnectedParticles.erase(spring->GetParticleA() );
	
	springs.erase(springs.begin()+index);
	collisionBehaviorNeedsUpdate=true;
	return this;
}

QMesh *QMesh::RemoveMatchingSprings(QParticle *particle)
{
	int i=0;
	while(i<springs.size()){
		QSpring *spring=springs[i];
		if(spring->GetParticleA()==particle || spring->GetParticleB()==particle){
			RemoveSpringAt(i);
		}else{
			++i;
		}
	}

	return this;
}

//UV Maps

QMesh *QMesh::AddUVMap(vector<int> map)
{
	UVMaps.push_back(map);
    return this;
}

QMesh *QMesh::RemoveUVMapAt(int index)
{
	UVMaps.erase(UVMaps.begin()+index );
    return this;
}

QMesh *QMesh::ClearUVMaps()
{
	UVMaps.clear();
    return this;
}

QMesh *QMesh::RemoveMatchingUVMaps(int particleIndex)
{
	int i=0;
	while(i<UVMaps.size() ){
		vector<int> map=UVMaps[i];
		int matchedIndex=-1;
		for(int n=0;n<map.size();++n){
			if( map[n]==particleIndex){
				matchedIndex=n;
			}
		}
		if(matchedIndex!=-1){
			if (map.size()>3){
				UVMaps[i].erase(UVMaps[i].begin()+matchedIndex);
			}else{
				RemoveUVMapAt(i);
				continue;
			}
			
		}
		i+=1;
	}
    return this;
}



QMesh *QMesh::CreateWithCircle(float radius, QVector centerPosition)
{
    QMesh * res=new QMesh();
	QParticle *particle=new QParticle(centerPosition.x,centerPosition.y,radius);
	res->AddParticle(particle);

	return res;
}

QMesh *QMesh::CreateWithPolygon(float radius,int sideCount,QVector centerPosition,int polarGrid,bool enableSprings, bool enablePolygons, float particleRadius){
	QMesh::MeshData meshData=GeneratePolygonMeshData(radius,sideCount,centerPosition,polarGrid,particleRadius);
	QMesh * res=QMesh::CreateWithMeshData(meshData,enableSprings,enablePolygons);
	return res;
}

QMesh *QMesh::CreateWithRect(QVector size,QVector centerPosition,QVector grid,bool enableSprings, bool enablePolygons, float particleRadius){
	QMesh::MeshData meshData=GenerateRectangleMeshData(size,centerPosition,grid,particleRadius);
	QMesh *res=CreateWithMeshData(meshData,enableSprings,enablePolygons);

	return res;
}

QMesh *QMesh::CreateWithMeshData(MeshData &data,bool enableSprings, bool enablePolygons)
{
	QMesh *res=new QMesh();
	//Adding particles
	for(int i=0;i<data.particlePositions.size();i++){
		QParticle *particle=new QParticle(data.particlePositions[i]);
		particle->SetIsInternal(data.particleInternalValues[i]);
		particle->SetRadius(data.particleRadValues[i]);
		particle->SetEnabled(data.particleEnabledValues[i]);
		particle->SetIsLazy(data.particleLazyValues[i]);
		res->AddParticle( particle );
	}

	//Adding polygon
	vector<QParticle*> polygonFromData;
	if(enablePolygons){
		for(int i=0;i<data.polygon.size();i++){
			polygonFromData.push_back( res->particles[ data.polygon[i] ] );
		}
	}
	if (polygonFromData.size()>0 )
		res->SetPolygon(polygonFromData);
	


	//Adding springs
	if(enableSprings){
		//Boundary springs
		for(int i=0;i<data.springList.size();i++){
			pair<int,int > springIndexes=data.springList[i];
			QSpring *spring=new QSpring( res->particles[springIndexes.first],res->particles[springIndexes.second],false);
			res->AddSpring(spring);
		}
		//Internal springs
		for(int i=0;i<data.internalSpringList.size();i++){
			pair<int,int > springIndexes=data.internalSpringList[i];
			QSpring *spring=new QSpring( res->particles[springIndexes.first],res->particles[springIndexes.second],true);
			res->AddSpring(spring);
		}
	}
	//Adding UV Maps
	res->UVMaps=data.UVMaps;

	res->position=data.position;
	res->rotation=data.rotation;
	return res;

}


vector<QMesh::MeshData> QMesh::GetMeshDatasFromFile(string filePath) {
	std::ifstream file;
	file.open(filePath);
	if(file.fail()){
		cout<< "QuarkPhysics Error: The file doesn't exist! | QMesh::GetMeshDatasFromFile";
		return vector<QMesh::MeshData>();
	}
	stringstream buffer;
	buffer<<file.rdbuf();
	string dataStr=buffer.str();

	return GetMeshDatasFromJsonData(dataStr);

}

vector<QMesh::MeshData> QMesh::GetMeshDatasFromJsonData(std::string &jsonBasedData) {
	json jsonData=json::parse(jsonBasedData);

	vector<QMesh::MeshData> result;

	auto meshes=jsonData["meshes"];

	for (auto &mesh: meshes){
		QMesh::MeshData meshData;
		if (mesh.contains("particles") ){

			auto particles=mesh["particles"];
			for (auto particle:particles){
				if ( particle.contains("position") )
					meshData.particlePositions.push_back(QVector( particle["position"][0], particle["position"][1]) );
				else
					meshData.particlePositions.push_back(QVector::Zero() );

				if ( particle.contains("radius") )
					meshData.particleRadValues.push_back(particle["radius"] );
				else
					meshData.particleRadValues.push_back(0.5f);

				if ( particle.contains("is_internal") )
					meshData.particleInternalValues.push_back(particle["is_internal"] );
				else 
					meshData.particleInternalValues.push_back(false );

				if ( particle.contains("enabled") )
					meshData.particleEnabledValues.push_back(particle["enabled"] );
				else 
					meshData.particleEnabledValues.push_back(true );

				if ( particle.contains("lazy") )
					meshData.particleLazyValues.push_back(particle["lazy"] );
				else 
					meshData.particleLazyValues.push_back(false );
			}
			if (mesh.contains("springs") ){
				auto springs=mesh["springs"];
				for (auto spring:springs){
					meshData.springList.push_back(pair<int,int>(spring[0],spring[1]) );
				}
			}

			if (mesh.contains("internal_springs") ){
				auto internalSprings=mesh["internal_springs"];
				for (auto spring:internalSprings){
					meshData.internalSpringList.push_back(pair<int,int>(spring[0],spring[1]) );
				}
			}

			if(mesh.contains("polygon") ){
				vector<int> polygonParticleIndexes=mesh["polygon"];
				if (polygonParticleIndexes.size()>0) {
					meshData.polygon=polygonParticleIndexes;
				}
			}

			if(mesh.contains("uv_maps") ){
				auto UVMapList=mesh["uv_maps"];

				for (auto map:UVMapList){
					vector<int> nmap;
					for (size_t i=0;i<map.size();++i ){
						int p=map[i];
						nmap.push_back(p);
					}
					meshData.UVMaps.push_back(nmap);
				}

			}

		}
		
		
		meshData.position=QVector(mesh["position"][0],mesh["position"][1] );
		meshData.rotation=mesh["rotation"];
		//Convert degrees to radians
		meshData.rotation*=(M_PI/180.0f);
		result.push_back(meshData);
	}

	return result;


}

float QMesh::GetPolygonArea(vector<QParticle *> &polygonPoints,bool withLocalPositions){
	float area=0;
	for(int i=0;i<polygonPoints.size();i++){
		QVector pA,pB;
		if(withLocalPositions){
			pA=polygonPoints[i]->GetPosition();
			pB=polygonPoints[ (i+1)%polygonPoints.size() ]->GetPosition();
		}else{
			pA=polygonPoints[i]->GetGlobalPosition();
			pB=polygonPoints[ (i+1)%polygonPoints.size() ]->GetGlobalPosition();
		}


		float h=pB.x-pA.x;
		float a=pA.y;
		float b=pB.y;

		area+=(a+b)*h*0.5f;
	}

	return -area;
}

bool QMesh::CheckCollisionBehaviors(QMesh *meshA, QMesh *meshB, CollisionBehaviors firstBehavior, CollisionBehaviors secondBehavior){
	if(meshA->GetCollisionBehavior()==firstBehavior && meshB->GetCollisionBehavior()==secondBehavior)
		return true;
	if(meshB->GetCollisionBehavior()==firstBehavior && meshA->GetCollisionBehavior()==secondBehavior)
		return true;
	return false;
}

QMesh::MeshData QMesh::GenerateRectangleMeshData(QVector size,QVector centerPosition,QVector grid, float particleRadius)
{
	QMesh::MeshData res;
	QVector halfSize=size*0.5f;
	if(grid.x<=1 && grid.y<=1){
		res.particlePositions.push_back(QVector(-halfSize.x ,-halfSize.y )+centerPosition );
		res.particlePositions.push_back(QVector(halfSize.x ,-halfSize.y )+centerPosition );
		res.particlePositions.push_back(QVector(halfSize.x ,halfSize.y )+centerPosition );
		res.particlePositions.push_back(QVector(-halfSize.x ,halfSize.y )+centerPosition );


		res.particleRadValues={particleRadius,particleRadius,particleRadius,particleRadius};
		res.particleInternalValues={false,false,false,false};
		res.particleEnabledValues={true,true,true,true};
		res.particleLazyValues={false,false,false,false};


		for(int i=0;i<res.particlePositions.size();i++){
			res.polygon.push_back(i);
		}


		res.springList.push_back(pair<int,int>(0,1));
		res.springList.push_back(pair<int,int>(1,2));
		res.springList.push_back(pair<int,int>(2,3));
		res.springList.push_back(pair<int,int>(3,0));

		res.internalSpringList.push_back(pair<int,int>(0,2));
		res.internalSpringList.push_back(pair<int,int>(1,3));

		res.UVMaps.push_back( {0,1,3});
		res.UVMaps.push_back( {1,2,3});
	}else{
		QVector cellSize=size/grid;

		//Adding all particle coordinates
		for(int iy=0;iy<(int)grid.y+1;iy++){
			for(int ix=0;ix<(int)grid.x+1;ix++){
				QVector nPos=QVector(ix*cellSize.x,iy*cellSize.y);

				res.particlePositions.push_back((centerPosition-halfSize)+nPos);
				res.particleRadValues.push_back(particleRadius);

				if(ix==0 || ix==grid.x || iy==0 || iy==grid.y){
					res.particleInternalValues.push_back(false);
				}else{
					res.particleInternalValues.push_back(true);
				}

				res.particleEnabledValues.push_back(true);
				res.particleLazyValues.push_back(false);

			}

		}

		//Adding spring pairs
		for(int i=0;i<res.particlePositions.size();i++){
		   int gridX=i%(int)(grid.x+1);
		   int gridY=floor((i-gridX)/(grid.x+1));
		   //GD.Print("i : "+ i + " gridX: "+ gridX + " gridY: "+gridY );
		   //To right
		   if(gridX!=grid.x){
			   pair<int,int> spr=gridY==0 ? pair<int,int>(i,i+1):pair<int,int>(i+1,i);
			   if(gridY==0 || gridY==(int)grid.y)
				   res.springList.push_back( spr );
			   else{
				   res.internalSpringList.push_back( spr );
			   }
		   }
		   //To right cross down
		   if(gridX!=grid.x && gridY!=(int)grid.y){
			   res.internalSpringList.push_back( pair<int,int>(i,i+grid.x+2) );
		   }
		   //To left cross down
		   if(gridX!=0 && gridY!=(int)grid.y){
			   res.internalSpringList.push_back( pair<int,int>(i,i+grid.x) );
		   }
		   // To down
		   if(gridY!=grid.y){
			   pair<int,int> spr=gridX==0 ? pair<int,int>(i+grid.x+1,i):pair<int,int>(i,i+grid.x+1);
			   if(gridX==0 || gridX==(int)grid.x){
				   res.springList.push_back(spr);
			   }else{
				   res.internalSpringList.push_back( spr );
			   }
		   }
	   }

		//Ordering boundary springs
		if(res.springList.size()>0){
		   vector<pair<int,int>> springListTemp(res.springList);
		   vector<pair<int,int>> orderedSprings;
		   orderedSprings.push_back( springListTemp[0] );
		   springListTemp.erase(springListTemp.begin());
		   int loopCount=0;
		   while(springListTemp.size()>0){
			   if(springListTemp.size()==1){
				   orderedSprings.push_back( springListTemp[0] );
				   springListTemp.clear();
				   break;
			   }
			   int i=0;
			   while(i<springListTemp.size()){
				   if(orderedSprings[orderedSprings.size()-1].second==springListTemp[i].first){
					   orderedSprings.push_back(springListTemp[i] );
					   springListTemp.erase(springListTemp.begin()+i);
					   break;
				   }
				   i+=1;
			   }


			   if(loopCount>res.springList.size()*res.springList.size())
				   break;
			   loopCount+=1;

		   }
		   res.springList=orderedSprings;
		}

		//Creating a polygon with boundary springs
		vector<int> polygon;
		for(int i=0;i<res.springList.size();i++){
			res.polygon.push_back(res.springList[i].first);
		}

		//Adding uv maps
		for(int iy=0;iy<(int)grid.y;iy++){
			for(int ix=0;ix<(int)grid.x;ix++){
				int currentIndex=iy*(grid.x+1)+ix;
				vector<int> triA;
				triA.push_back(currentIndex);
				triA.push_back(currentIndex+1);
				triA.push_back(currentIndex+grid.x+1);
				res.UVMaps.push_back(triA);

				vector<int> triB;
				triB.push_back(currentIndex+1);
				triB.push_back(currentIndex+grid.x+2);
				triB.push_back(currentIndex+1+grid.x);
				
				res.UVMaps.push_back(triB);

			}

		}

	}

	return res;
}

QMesh::MeshData QMesh::GeneratePolygonMeshData(float radius, int sideCount, QVector centerPosition, int polarGrid, float particleRadius)
{
	QMesh::MeshData res;

	//Boundary Particle, Polygon and Springs
	float anglePart=(M_PI*2)/sideCount;
	vector<int> polygon;
	for(int i=0;i<sideCount;i++){
		float curAng=anglePart*i;
		QVector curNorm(cos(curAng),sin(curAng) );
		QVector nPos=centerPosition+curNorm*radius;
		res.particlePositions.push_back(nPos);
		res.particleRadValues.push_back(particleRadius);
		res.particleInternalValues.push_back(false);
		res.particleEnabledValues.push_back(true);
		res.particleLazyValues.push_back(false);
		res.polygon.push_back(i);
		res.springList.push_back( pair<int,int>(i,(i+1)%sideCount) );

	}
	

	


	

	int centerParticleFactor=0;

	//Internal Particle And Spring
	if(polarGrid>0){
		float radiusPart=radius/polarGrid;
		for(int i=polarGrid-1;i>0;i--){
			float curRadius=radiusPart*i;
			for(int n=0;n<sideCount;n++){
				float curAng=anglePart*n;
				QVector curNorm(cos(curAng),sin(curAng) );
				QVector nPos=centerPosition+curNorm*curRadius;
				res.particlePositions.push_back(nPos);
				res.particleRadValues.push_back(particleRadius);
				res.particleInternalValues.push_back(true);
				res.particleEnabledValues.push_back(true);
				res.particleLazyValues.push_back(false);

				if(n!=0)
					res.internalSpringList.push_back( pair<int,int>(res.particlePositions.size()-2,res.particlePositions.size()-1) );
			}
			res.internalSpringList.push_back( pair<int,int>(res.particlePositions.size()-1,res.particlePositions.size()-sideCount) );


			//Adding Internal Spring
			for(int n=res.particlePositions.size()-sideCount;n<res.particlePositions.size();n++){
				int a=n-sideCount;
				int b=n==res.particlePositions.size()-1 ? res.particlePositions.size()-sideCount*2:n-(sideCount-1);
				int c=n==res.particlePositions.size()-1 ? res.particlePositions.size()-sideCount:n+1;
				int d=n;
				res.internalSpringList.push_back( pair<int,int>(d,a) );
				res.internalSpringList.push_back( pair<int,int>(d,b) );
				res.internalSpringList.push_back( pair<int,int>(c,a) );
				res.internalSpringList.push_back( pair<int,int>(c,b) );
			}
		}

		//Adding a Center Particle
		centerParticleFactor=1;
		res.particlePositions.push_back(centerPosition );
		res.particleRadValues.push_back(particleRadius);
		res.particleInternalValues.push_back(true);
		res.particleEnabledValues.push_back(true);
		res.particleLazyValues.push_back(false);
		for(int i=res.particlePositions.size()-sideCount-1;i<res.particlePositions.size()-1;i++){
			res.internalSpringList.push_back(pair<int,int>( res.particlePositions.size()-1,i ) );
		}
		/* for(int i=0;i<sideCount-1;i++){
			res.internalSpringList.push_back(pair<int,int>( res.particlePositions.size()-1,i ) );
		} */

		
	}

	//Adding construction springs
	if (polarGrid>=0){
		int pc=res.particlePositions.size();
		int startIndex=pc-sideCount-centerParticleFactor;
		for(int i=0;i<sideCount;i++){
			int prevParticle=startIndex+(( i-2+sideCount)%sideCount );
			int particle=startIndex+i;
			int nextParticle=startIndex+( (i+2)%sideCount );
			res.internalSpringList.push_back( pair<int,int>(prevParticle,particle));
			res.internalSpringList.push_back( pair<int,int>(particle,nextParticle));

		}
	}

	


	//Adding UV Maps

	if(polarGrid<=0){
		vector<int> map;
		for(size_t i=0;i<res.polygon.size();++i){
			map.push_back(i);
		}
		res.UVMaps.push_back(map);
	}else if(polarGrid==1){
		for(size_t i=0;i<res.polygon.size();++i){
			vector<int> map;
			map.push_back( i );
			map.push_back( (i+1)%res.polygon.size() );
			map.push_back(res.particlePositions.size()-1 );
			res.UVMaps.push_back(map);
		}
	}else{
		for(size_t i=0;i<res.particlePositions.size()-res.polygon.size()-1;++i){
			int a=i;
			int b=(i+1)%res.polygon.size()==0 ? (i+1)-res.polygon.size() : i+1; //next i
			int c=b+res.polygon.size();
			int d=i+res.polygon.size();
			
			vector<int> triA;
			triA.push_back( a );
			triA.push_back( b );
			triA.push_back( d );
			res.UVMaps.push_back(triA);

			vector<int> triB;
			triB.push_back( b );
			triB.push_back( c );
			triB.push_back( d );
			res.UVMaps.push_back(triB);
		}
		for(size_t i=res.particlePositions.size()-res.polygon.size()-1;i<res.particlePositions.size()-1;++i){
			vector<int> map;
			int a=i;
			int b=(i+1)%res.polygon.size()==0 ? (i+1)-res.polygon.size() : i+1; //next i
			int c=res.particlePositions.size()-1;

			map.push_back( a );
			map.push_back( b );
			map.push_back( c );
			res.UVMaps.push_back(map);
		}

	}


	return res;
}


pair<QVector, float> QMesh::GetAveragePositionAndRotation(vector<QParticle *> particleCollection)
{
    if(particleCollection.size()==1)
		return pair<QVector, float>(particleCollection[0]->GetGlobalPosition(),0.0f);
	//Finding Actual Position
	QVector averagePosition=QVector::Zero();
	
	for(int i=0;i<particleCollection.size();i++){
		QParticle *particle=particleCollection[i];
		averagePosition+=particle->GetGlobalPosition();
	}  
	averagePosition/=particleCollection.size();
	
	float averageRotation=0;
	float cosAxis=0.0f;
	float sinAxis=0.0f;
	for(int i=0;i<particleCollection.size();i++){
		QParticle *particle=particleCollection[i];
		
		QVector currentVec=particle->GetGlobalPosition()-averagePosition;
		cosAxis+=currentVec.Dot(particle->GetPosition() );
		sinAxis+=currentVec.Dot(particle->GetPosition().Perpendicular() );
	}

	
	float rad=atan2(sinAxis,cosAxis);
	averageRotation=rad;
	

	return pair< QVector, float >(averagePosition,averageRotation);
}

vector<QVector> QMesh::GetMatchingParticlePositions(vector<QParticle *> particleCollection, QVector targetPosition, float targetRotation)
{
    QVector localCenterPosition;
	QVector globalCenterPosition;
	for(auto particle:particleCollection){
		localCenterPosition+=particle->GetPosition();
	}
	localCenterPosition/=particleCollection.size();

	vector<QVector> positions;
	for(int n=0;n<particleCollection.size();n++){
		QParticle * particle=particleCollection[n];
		
		QVector targetPos=(particle->GetPosition()-localCenterPosition).Rotated(-targetRotation);
		targetPos+=targetPosition;
		//particle->GetOwnerMesh()->GetOwnerBody()->GetWorld()->GetGizmos()->push_back(new QGizmoCircle(targetPos,3.0f) );
		positions.push_back(targetPos);
	}

	return positions;
}

