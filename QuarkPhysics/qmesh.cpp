
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


QMesh::QMesh(){
}

QMesh::~QMesh()
{
	for(int i=0;i<particles.size();i++){
		if (particles[i]!=nullptr){
			delete particles[i];
			particles[i]=nullptr;
		}
	}
	particles.clear();

	for(int i=0;i<springs.size();i++){
		if(springs[i]!=nullptr){
			delete springs[i];
			springs[i]=nullptr;
		}
	}
	springs.clear();

}

void QMesh::UpdateCollisionBehavior()
{
	if(ownerBody==nullptr)
		return;
	

	if(GetPolygonParticleCount()>0){
		if(ownerBody->simulationModel==QBody::SimulationModels::RIGID_BODY){
			collisionBehavior=CollisionBehaviors::POLYGONS;
		}else{
			collisionBehavior=CollisionBehaviors::POLYLINE;
		}

	}else{
		collisionBehavior=CollisionBehaviors::CIRCLES;
	}
}



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
	particles.erase(particles.begin()+index);
	if(ownerBody!=nullptr){
		if (ownerBody->mode==QBody::Modes::STATIC)
			ownerBody->UpdateMeshTransforms();
		ownerBody->inertiaNeedsUpdate=true;
		ownerBody->circumferenceNeedsUpdate=true;
	}
	collisionBehaviorNeedsUpdate=true;
	return this;
}

QMesh *QMesh::RemoveParticle(QParticle *particle){
	int index=-1;
	for(int i=0;i<particles.size();i++){
		if(particles[i]==particle){
			index=i;
			break;
		}
	}
	RemoveParticleAt(index);
	return this;
}

int QMesh::GetParticleCount(){
	return particles.size();
}

QParticle *QMesh::GetParticleAt(int index){
	return particles[index];
}

QMesh *QMesh::SetPolygon(vector<QParticle *> polygonParticles)
{
	polygon=polygonParticles;

	subConvexPolygonsNeedsUpdate=true;

	collisionBehaviorNeedsUpdate=true;

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
    return this;
}

QMesh *QMesh::RemovePolygon()
{
	polygon.clear();
	subConvexPolygons.clear();
	collisionBehaviorNeedsUpdate=true;
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

void QMesh::UpdateSubConvexPolygons()
{
	subConvexPolygons.clear();
	if (CheckIsPolygonConcave(polygon)==true ){
		DecompositePolygon(polygon,subConvexPolygons);
		// cout<<"polygon is concave"<<endl;
	}else{
		subConvexPolygons.push_back( polygon );
		// cout<<"polygon is convex"<<endl;
	}
	
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
	if(polygonIntersection==true){
		//cout<<"there is line intersection in polygon"<<endl;
		pair<QVector,float> averagePosRot=QMesh::GetAveragePositionAndRotation(polygon);
		vector<QVector> matchingShape=QMesh::GetMatchingParticlePositions(polygon,averagePosRot.first,averagePosRot.second);
		for(int i=0;i<matchingShape.size();i++ ){
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

			
			
			pp->SetGlobalPosition( p->GetGlobalPosition()+toPrev.Rotated(angularForce) );
			np->SetGlobalPosition( p->GetGlobalPosition()+toNext.Rotated(-angularForce) );

			
			
		}

		if(angleRad<minAngle){
			float diffAngle=minAngle-angleRad;
			float angularForce=diffAngle*0.5f;

			pp->SetGlobalPosition( p->GetGlobalPosition()+toPrev.Rotated(angularForce) );
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

	
	while (indexList.size()>3 ){
		
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

void QMesh::DecompositePolygon(vector<QParticle *> &polygonParticles, vector<vector<QParticle *>> &polygons)
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



QMesh *QMesh::AddSpring(QSpring *spring)
{
	springs.push_back(spring);
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

QMesh *QMesh::CreateWithCircle(float radius, QVector centerPosition){
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
			res->springs.push_back(spring);
		}
		//Internal springs
		for(int i=0;i<data.internalSpringList.size();i++){
			pair<int,int > springIndexes=data.internalSpringList[i];
			QSpring *spring=new QSpring( res->particles[springIndexes.first],res->particles[springIndexes.second],true);
			res->springs.push_back(spring);
		}
	}
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
		auto particles=mesh["particles"];
		for (auto particle:particles){
			meshData.particlePositions.push_back(QVector( particle["position"][0], particle["position"][1]) );
			meshData.particleRadValues.push_back(particle["radius"] );
			meshData.particleInternalValues.push_back(particle["is_internal"] );
		}
		auto springs=mesh["springs"];
		for (auto spring:springs){
			meshData.springList.push_back(pair<int,int>(spring[0],spring[1]) );
		}

		auto internalSprings=mesh["internal_springs"];
		for (auto spring:internalSprings){
			meshData.internalSpringList.push_back(pair<int,int>(spring[0],spring[1]) );
		}

		vector<int> polygonParticleIndexes=mesh["polygon"];
		if (polygonParticleIndexes.size()>0) {
			meshData.polygon=polygonParticleIndexes;
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

	return abs(area);
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


		for(int i=0;i<res.particlePositions.size();i++){
			res.polygon.push_back(i);
		}


		res.springList.push_back(pair<int,int>(0,1));
		res.springList.push_back(pair<int,int>(1,2));
		res.springList.push_back(pair<int,int>(2,3));
		res.springList.push_back(pair<int,int>(3,0));

		res.internalSpringList.push_back(pair<int,int>(0,2));
		res.internalSpringList.push_back(pair<int,int>(1,3));
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
		res.polygon.push_back(i);
		res.springList.push_back( pair<int,int>(i,(i+1)%sideCount) );

	}
	

	if(polarGrid<0)
		return res;


	//Adding construction springs
	int pc=res.particlePositions.size();
	for(int i=0;i<pc;i++){
		int prevParticle=(i-2+pc)%pc;
		int particle=i;
		int nextParticle=(i+2)%pc;
		res.internalSpringList.push_back( pair<int,int>(prevParticle,particle));
		res.internalSpringList.push_back( pair<int,int>(particle,nextParticle));

	}

	if(polarGrid<=0)
		return res;



	//Internal Particle And Spring
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
	if(polarGrid>0){
		res.particlePositions.push_back(centerPosition );
		res.particleRadValues.push_back(particleRadius);
		res.particleInternalValues.push_back(true);
		for(int i=res.particlePositions.size()-sideCount-1;i<res.particlePositions.size()-1;i++){
			res.internalSpringList.push_back(pair<int,int>( res.particlePositions.size()-1,i ) );
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
	for(auto particle:particleCollection){
		localCenterPosition+=particle->GetPosition();
	}
	localCenterPosition/=particleCollection.size();

	vector<QVector> positions;
	for(int n=0;n<particleCollection.size();n++){
		QParticle * particle=particleCollection[n];
		
		QVector targetPos=(particle->GetPosition()-localCenterPosition).Rotated(-targetRotation);
		targetPos+=targetPosition;
		//world->GetGizmos()->push_back(new QGizmoCircle(targetPos,3.0f) );
		positions.push_back(targetPos);
	}

	return positions;
}

