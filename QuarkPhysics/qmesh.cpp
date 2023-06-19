
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


QMesh::QMesh(){
}

QMesh::~QMesh()
{
	for(int i=0;i<particles.size();i++){
		delete particles[i];
	}
}

void QMesh::UpdateCollisionBehavior()
{
	if(ownerBody==nullptr)
		return;
	if(closedPolygons.size()>0){
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
		ownerBody->inertiaNeedsUpdate=true;
		ownerBody->circumferenceNeedsUpdate=true;
	}
	collisionBehaviorNeedsUpdate=true;
	return this;
}

QMesh *QMesh::RemoveParticleAt(int index){
	QParticle *particle=particles[index];
	RemoveMatchingClosedPolygons(particle);
	RemoveMatchingSprings(particle);
	particles.erase(particles.begin()+index);
	if(ownerBody!=nullptr){
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

QMesh *QMesh::AddClosedPolygon(vector<QParticle *> polygon)
{
	closedPolygons.push_back(polygon);
	if(ownerBody!=nullptr){
		ownerBody->inertiaNeedsUpdate=true;
		ownerBody->circumferenceNeedsUpdate=true;
	}
	collisionBehaviorNeedsUpdate=true;
	return this;
}

QMesh *QMesh::RemoveClosedPolygonAt(int index)
{
	closedPolygons.erase(closedPolygons.begin()+index);
	if(ownerBody!=nullptr){
		ownerBody->inertiaNeedsUpdate=true;
		ownerBody->circumferenceNeedsUpdate=true;
	}
	collisionBehaviorNeedsUpdate=true;
	return this;

}

QMesh *QMesh::RemoveMatchingClosedPolygons(QParticle *particle)
{
	int i=0;
	while(i<closedPolygons.size()){
		vector<QParticle*> &polygon=closedPolygons[i];
		bool matched=false;
		int n=0;
		while(n<polygon.size()){
			if(polygon[n]==particle){
				polygon.erase(polygon.begin()+n);
				matched=true;
			}else{
				++n;
			}
		}
		if(matched==true && polygon.size()<3){
			RemoveClosedPolygonAt(i);

		}else{
			++i;
		}
	}
	return this;
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

	//Adding closed polygons
	if(enablePolygons){
		for(int i=0;i<data.closedPolygonList.size();i++){
			vector<int> polygonIndexes=data.closedPolygonList[i];
			vector<QParticle *> polygon;
			for(int n=0;n<polygonIndexes.size();n++){
				polygon.push_back( res->particles[ polygonIndexes[n] ] );
			}
			res->closedPolygons.push_back(polygon);
		}
	}


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
		auto polygons=mesh["polygons"];
		for (auto polygon:polygons){
			meshData.closedPolygonList.push_back(polygon);
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

		vector<int> polygon;
		for(int i=0;i<res.particlePositions.size();i++){
			polygon.push_back(i);
		}
		res.closedPolygonList.push_back(polygon);


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

		//Creating a closed polygon with boundary springs
		vector<int> polygon;
		for(int i=0;i<res.springList.size();i++){
			polygon.push_back(res.springList[i].first);
		}
		res.closedPolygonList.push_back(polygon);

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
		polygon.push_back(i);
		res.springList.push_back( pair<int,int>(i,(i+1)%sideCount) );

	}
	res.closedPolygonList.push_back(polygon);

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




