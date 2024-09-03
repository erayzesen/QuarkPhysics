
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

#include "qcollision.h"
#include <cmath>
#include <iostream>
#include "qmesh.h"
#include "qworld.h"
#include "qgizmos.h"
#include "qaabb.h"




QObjectPool<QCollision::Contact> QCollision::contactPool(100,50);




void QCollision::PolylineAndPolygon(vector<QParticle*> &polylineParticles, vector<QParticle*> &polygonParticles, vector<Contact*> &contacts)
{
	//CircleAndPolygon(polylineParticles,polygonParticles,contacts);



	/*
   A. Start the loop for all segments of polyline
   B. Start the loop for all particles of polygon
   C. Check an intersection between the segment of polyline and the virtual vector between the polygon particle and the polygon center
   D. If the intersection is exist, calculate the distance between the particle of polygon and the segment of polyline
   */



	//Get angle bisector list of the polygon

	vector<QVector> bisectorList;
	float maxRayLength=QWorld::MAX_WORLD_SIZE;
	for(int i=0;i<polygonParticles.size();i++){
		int pi=(i-1+polygonParticles.size())%polygonParticles.size(); //previous index
		int ni=(i+1)%polygonParticles.size(); //next index
		QParticle *pp=polygonParticles[pi];
		QParticle *p=polygonParticles[i];
		QParticle *np=polygonParticles[ni];

		
		QVector bisectorUnit=QVector::GeteBisectorUnitVector(pp->GetGlobalPosition(),p->GetGlobalPosition(),np->GetGlobalPosition(),true );
		

		QVector bisectorRay=bisectorUnit*QWorld::MAX_WORLD_SIZE;

		
		int sia=ni; // segment index a
		QVector bisectorVector=QVector::Zero();
		if(polylineParticles==polygonParticles){
			
			float rayLength=abs((p->GetGlobalPosition()-pp->GetGlobalPosition() ).Dot(bisectorUnit ) )*0.5f;
			
			bisectorVector=bisectorUnit*rayLength;
		}else{
			
			float minDistance=QWorld::MAX_WORLD_SIZE;
			bool findedIntersection=false;
			while(sia!=pi){
				int sib=(sia+1)%polygonParticles.size(); //segment index b
				QVector intersectionPoint=LineIntersectionLine(p->GetGlobalPosition(),p->GetGlobalPosition()+bisectorRay,polygonParticles[sia]->GetGlobalPosition(),polygonParticles[sib]->GetGlobalPosition());
				if(intersectionPoint.isNaN()==false){
					QVector findedVec=intersectionPoint-p->GetGlobalPosition();
					float distance=findedVec.Length();
					if(distance<minDistance){
						bisectorVector=findedVec*0.5f;
						minDistance=distance;
					}
					findedIntersection=true;
				}
				sia=sib;
			}
		}

		

		

		bisectorList.push_back(bisectorVector );
		//p->GetOwnerMesh()->GetOwnerBody()->GetWorld()->gizmos.push_back(new QGizmoLine(p->GetGlobalPosition(),p->GetGlobalPosition()+bisectorVector,true) );

		



	}

	//A. Start the loop for all segments of the polyline


	for(int i=0;i<polylineParticles.size();i++){

		
		QParticle *s1=polylineParticles[i];
		QParticle *s2=polylineParticles[ (i+1)%polylineParticles.size() ];

		QVector s1Pos=s1->GetGlobalPosition();
		QVector s2Pos=s2->GetGlobalPosition();

		QVector normal=(s2Pos-s1Pos).Normalized().Perpendicular();

		//add radius factor to the corner
		if(s1->GetRadius()>0.5f){
			s1Pos+=normal*s1->GetRadius();
		}
		if(s2->GetRadius()>0.5f){
			s2Pos+=normal*s2->GetRadius();
		}

		normal=(s2Pos-s1Pos).Normalized().Perpendicular();

		//B. Start the loop for all particles of the polygon

		for(int n=0;n<polygonParticles.size();n++){
			if (bisectorList[n]==QVector::Zero() )
				continue;
			QParticle *p=polygonParticles[n];
			//Check particles for self collisions
			//if (p==s1 || p==s2) continue;

			QVector pPos=p->GetGlobalPosition(); 
			if(p->GetRadius()>0.5f){
				pPos-=p->GetRadius()*normal;
			}
			

			//C. Check an intersection between the segment of polyline and the virtual vector
			QVector intersection=LineIntersectionLine(pPos,pPos+bisectorList[n],s1Pos,s2Pos);
			if( intersection.isNaN() )continue;


			//p->GetOwnerMesh()->GetOwnerBody()->GetWorld()->gizmos.push_back( new QGizmoLine(pPos,pPos+bisectorList[n],true) );


			//D. If the intersection is exist, calculate the distance between the particle of polygon and the segment of polyline
			QVector bridgeVec=pPos-s1Pos;

			float penetration=bridgeVec.Dot(-normal);
			
			
			
			QCollision::Contact *contact=QCollision::GetContactPool().Create().data;
			contact->Configure(p,p->GetGlobalPosition(),normal,penetration,vector<QParticle*>{s1,s2});
			contacts.push_back(contact);

		}
	}

}

void QCollision::CircleAndPolyline(vector<QParticle *> &circleParticles, vector<QParticle *> &polylineParticles,QAABB polylineAABB,vector<QCollision::Contact*> &contacts,bool circlesArePolygon)
{

	/* The algorithm detects collisions between one or more particles and a polyline. 
	It utilizes point-polygon tests, ray casting, and circle-line collision tests if conditions require.

	A. A loop is performed for each circle particle and subjected to testing. 
	B. If the circular particle belongs to a collection with 3 or more elements, a ray vector is prepared towards the angle bisector using its edges. 
	C. Collision tests are applied between the circle particle and the nearest polyline particle to find the colliding edge.
		C-1. If the circle particle belongs to a collection with 3 or more elements, a ray intersection test is applied from the particle to the two edges
		 of the polyline to find the reference surface.
		C-2. If the circle particle belongs to a collection with fewer than 3 points, the closest edge is determined using a vertical projection.
	D. If the circle particle is not within the polyline and the radius value is greater than 0.5, a collision edge is queried with a distance 
	test between the circle diameter and the edges of the nearest particle.
	*/
	

	vector< vector<QParticle*> > nearestSides;
	
	//A. A loop is performed for each circle particle and subjected to testing. 
	for (size_t ia=0;ia<circleParticles.size();ia++ ){

		QParticle * pA=circleParticles[ia];
		
		QVector particleSize=QVector(pA->GetRadius(),pA->GetRadius() );
		QAABB particleAABB=QAABB(pA->GetGlobalPosition()-particleSize,pA->GetGlobalPosition()+particleSize );

		if(particleAABB.isCollidingWith(polylineAABB)==false ){
			continue;
		}

		

		

		int collidedSideIndex=-1;

		//Checking whether the cirlce particle is in the polyline. 
		bool collisionIsPolygonal=false;
		if (circlesArePolygon){
			if (circleParticles!=polylineParticles){
				if (PointInPolygonWN(pA->GetGlobalPosition(),polylineParticles)){
					collisionIsPolygonal=true;
				}else{
					//intersection tests between target particle sides and polyline
					if(circleParticles.size()>3 && polylineParticles.size()>3 ){
						QParticle *ppA=circleParticles[ (ia-1+circleParticles.size() )%circleParticles.size() ];
						QParticle *npA=circleParticles[ (ia+1 )%circleParticles.size() ];
						for(size_t j=0;j<polylineParticles.size();++j ){
							QParticle * pJ=polylineParticles[j];
							QParticle *npJ=polylineParticles[ (j+1)% polylineParticles.size() ];
							bool sideIntersectionA=LineIntersectionLine(ppA->GetGlobalPosition(),pA->GetGlobalPosition(),pJ->GetGlobalPosition(), npJ->GetGlobalPosition() ).isNaN()==false;
							
							if( sideIntersectionA){
								bool sideIntersectionB=LineIntersectionLine(pA->GetGlobalPosition(),npA->GetGlobalPosition(),pJ->GetGlobalPosition(), npJ->GetGlobalPosition() ).isNaN()==false;
								if (sideIntersectionB){
									collisionIsPolygonal=true;
								}
								
							}
						}

					}
				}
		}
		

		}


		
		if (collisionIsPolygonal) {

			//B. If the circular particle belongs to a collection with 3 or more elements, a ray vector is prepared towards the angle bisector using its edges.
			nearestSides.clear();
			QVector rayEndPoint=QVector::Zero();
			QVector rayUnit;
			if(circleParticles.size()>=3){
				QParticle *prevParticle=circleParticles[ (ia-1+circleParticles.size() )%circleParticles.size() ];
				QParticle *nextParticle=circleParticles[ (ia+1 )%circleParticles.size() ];
				rayUnit=QVector::GeteBisectorUnitVector(prevParticle->GetGlobalPosition(), pA->GetGlobalPosition(), nextParticle->GetGlobalPosition() );
				rayEndPoint=pA->GetGlobalPosition()+rayUnit*QWorld::MAX_WORLD_SIZE;
				
			}

			//C. Collision tests are applied between the circle particle and the nearest polyline particle to find the colliding edge.

			

			int ni=FindNearestParticleOfPolygon(pA,polylineParticles );
			
			QParticle *pB=polylineParticles[ni];

			nearestSides.push_back(vector<QParticle*>{ polylineParticles[ (ni-1+polylineParticles.size() )%polylineParticles.size() ], pB}  );
			nearestSides.push_back(vector<QParticle*>{ pB, polylineParticles[ (ni+1)%polylineParticles.size() ]}  );

			//Nearest particle is on the outside of the test particle sides.
			if (circlesArePolygon){
				bool isNearesParticleOnWrongSide=true;

				for (size_t j=0;j<nearestSides.size();++j ){
					QVector sideVec=nearestSides[j][1]->GetGlobalPosition()-nearestSides[j][0]->GetGlobalPosition();
					QVector sidePerp=sideVec.Perpendicular();
					QVector bVector=pA->GetGlobalPosition()-nearestSides[j][0]->GetGlobalPosition();
					if(sidePerp.Dot(rayUnit)>0 ){
						isNearesParticleOnWrongSide=false;
					}
				}
				if(isNearesParticleOnWrongSide==true  ){
					nearestSides.clear();
					for(size_t j=0;j<polylineParticles.size();++j ){
						int nj=(j+1)%polylineParticles.size();

						QVector sideVec=polylineParticles[nj]->GetGlobalPosition()-polylineParticles[j]->GetGlobalPosition();
						QVector sidePerp=sideVec.Perpendicular();
						if(sidePerp.Dot(rayUnit)>0 ){
							nearestSides.push_back(vector<QParticle*>{ polylineParticles[ j], polylineParticles[ nj]}  );			
						}
					}
				}
			}


			

			float penetration=0;

			QVector normal;

			float minDistance=-QWorld::MAX_WORLD_SIZE;

			float maxIntersectionDistance=-QWorld::MAX_WORLD_SIZE;

			for( int n=0;n<nearestSides.size();n++ ){ 
				QParticle *sA=nearestSides[n][0];
				QParticle *sB=nearestSides[n][1];

				QVector sideVec=sB->GetGlobalPosition()-sA->GetGlobalPosition();
				QVector sideNormal=sideVec.Normalized().Perpendicular();

				QVector sAPos=sA->GetGlobalPosition();
				QVector sBPos=sB->GetGlobalPosition();
				
				
				if(sA->GetRadius()+sB->GetRadius()>1.0 ){
					if(sA->GetRadius()>0.5){
						sAPos+=sA->GetRadius()*sideNormal;
					}
					if(sB->GetRadius()>0.5){
						sBPos+=sB->GetRadius()*sideNormal;
					}

					sideVec=sBPos-sAPos;
					sideNormal=sideVec.Normalized().Perpendicular();
				}

				if (circleParticles.size()>=3 ){
					/*C-1. If the circle particle belongs to a collection with 3 or more elements, a ray intersection test is applied from the particle to the two edges
		 			of the polyline to find the reference surface.*/

					QVector intersection=LineIntersectionLine(rayEndPoint,pA->GetGlobalPosition(),sAPos,sBPos);

					if(intersection.isNaN()==false ){
						float radius=pA->GetRadius();
						QVector bridgeVec=pA->GetGlobalPosition()-sAPos;
						float dist=bridgeVec.Dot( sideNormal );
						float distIntersection=(intersection-pA->GetGlobalPosition()).Length();
						//pA->GetOwnerMesh()->GetOwnerBody()->GetWorld()->gizmos.push_back(new QGizmoLine(pA->GetGlobalPosition(),pA->GetGlobalPosition()+rayUnit*16,true ) );
						if(dist<0 && dist>minDistance){
							minDistance=dist;
							normal=sideNormal;
							penetration=dist-radius;
							collidedSideIndex=n;
							//pA->GetOwnerMesh()->GetOwnerBody()->GetWorld()->gizmos.push_back(new QGizmoLine(pA->GetGlobalPosition(),pA->GetGlobalPosition()+rayUnit*16,true ) );
							//pA->GetOwnerMesh()->GetOwnerBody()->GetWorld()->gizmos.push_back(new QGizmoLine(sAPos,sBPos,false ) );
						}
					}
				}else{
					//C-2. If the circle particle belongs to a collection with fewer than 3 points, the closest edge is determined using a vertical projection.

					QVector bridgeVec=pA->GetGlobalPosition()-sAPos;

					float dist=bridgeVec.Dot( sideNormal );

					float radius=pA->GetRadius();
					if (dist>minDistance && dist<radius){
						minDistance=dist;
						normal=sideNormal;
						penetration=dist-radius;
						collidedSideIndex=n;
					}
				}

								


			}

			
			if (collidedSideIndex==-1){
				continue;
			}

			

			//pA->GetOwnerMesh()->GetOwnerBody()->GetWorld()->gizmos.push_back(new QGizmoCircle(pA->GetGlobalPosition(),5.0f ) );

			
			QCollision::Contact *contact=QCollision::GetContactPool().Create().data;
			contact->Configure( pA,pA->GetGlobalPosition(),normal,-penetration,vector<QParticle*>{ nearestSides[collidedSideIndex][0],nearestSides[collidedSideIndex][1] }  );
			contacts.push_back(contact) ;
			

		}else{
			/* D. If the circle particle is not within the polyline and the radius value is greater than 0.5, a collision edge is queried with a distance 
			test between the circle diameter and the edges of the nearest particle. */
			if(pA->GetRadius()>0.5){
				
				pair<int,int> ns=FindNearestSideOfPolygon(pA->GetGlobalPosition(),polylineParticles,true );

				if(ns.first==-1 && ns.second==-1)
					continue;

				int ni=FindNearestParticleOfPolygon(pA,vector<QParticle*>{polylineParticles[ns.first],polylineParticles[ns.second] } );
				if (ni==0){
					ni=ns.first;
				}else{
					ni=ns.second;
				}

				QParticle *pB=polylineParticles[ni];

				
			

				nearestSides.push_back(vector<QParticle*>{ polylineParticles[ (ni-1+polylineParticles.size() )%polylineParticles.size() ], pB}  );
				nearestSides.push_back(vector<QParticle*>{ pB, polylineParticles[ (ni+1)%polylineParticles.size() ]}  );

				for(size_t is=0;is<nearestSides.size();is++ ){
					QParticle *s1=nearestSides[is][0];
					QParticle *s2=nearestSides[is][1];

					QVector s1Pos=s1->GetGlobalPosition();
					QVector s2Pos=s2->GetGlobalPosition();


					QVector segVec=(s2Pos-s1Pos);
					QVector unit=segVec.Normalized();
					QVector normal=unit.Perpendicular();

					if(s1->GetRadius()>0.5 || s2->GetRadius()>0.5){
						if(s1->GetRadius()>0.5){
							s1Pos+=s1->GetRadius()*normal;
						}
						if(s2->GetRadius()>0.5){
							s2Pos+=s2->GetRadius()*normal;
						}
						segVec=s2Pos-s1Pos;
						unit=segVec.Normalized();
						normal=unit.Perpendicular();
					}

					float len=segVec.Length();



					QVector bridgeVec=pA->GetGlobalPosition()-s1Pos;

					QVector testVirtualSegment[2];

					testVirtualSegment[0]=s1Pos;
					testVirtualSegment[1]=s2Pos;


					QVector testBridgeVec=pA->GetGlobalPosition()-testVirtualSegment[0];

					float perpProj=testBridgeVec.Dot(normal);

					if(abs(perpProj)<pA->GetRadius()){
						float proj=bridgeVec.Dot(unit);
						if(proj>=0 && proj<=len){
							int projSign=perpProj<0 ? -1:1;
							float penetration=abs( (pA->GetRadius()*projSign)-perpProj);
							QVector contactPosition=pA->GetGlobalPosition()-(pA->GetRadius()*projSign*normal);

							
							QCollision::Contact *contact=QCollision::GetContactPool().Create().data;
							contact->Configure(pA,contactPosition,normal,penetration,vector<QParticle*>{s1,s2});
							contacts.push_back(contact);

						}
					}

				}

			}
		}



			

	}
	
}

void QCollision::CircleAndCircle(vector<QParticle*> &particlesA,vector<QParticle*> &particlesB,QAABB boundingBoxB,vector<QCollision::Contact*> &contacts,float specifiedRadius){

	/*
	A. Start the loop for all points of particlesA
	B. Start the loop for all points of particlesB
	C. Check the distance between both points
	D. If the distance is less than radius of these points, apply the collision
	*/

	float totalRadius;
	float totalRadiusPow;
	float radiusA;
	float radiusB;
	QVector bboxSizeA;

	QVector distVec;
	float positionalPenetrationSq;
	float positionalPenetration;
	float penetration;
	QVector normal;
	QVector contactPosition;


	if(specifiedRadius!=0.0){
		radiusA=specifiedRadius;
		radiusB=specifiedRadius;
		totalRadius=specifiedRadius+specifiedRadius;
		totalRadiusPow=totalRadius*totalRadius;
		bboxSizeA=QVector(specifiedRadius,specifiedRadius);
	}

	//A. Start the loop for all points of circleparticlesA
	for(size_t i=0;i<particlesA.size();i++){
		QParticle *pA=particlesA[i];

		//Optimization phase: aabb test 
		if(particlesA.size()>1 && particlesB.size()>1 ){
			if(specifiedRadius==0.0){
				bboxSizeA=QVector(radiusA,radiusA);
			}
			QAABB boundingBoxA(pA->GetGlobalPosition()-bboxSizeA,pA->GetGlobalPosition()+bboxSizeA );

			if(boundingBoxA.isCollidingWith(boundingBoxB)==false ){
				continue;
			}
		}

		//Optimization phase: reducing loop count whether the test is self collision
		size_t n=0;
		if(particlesA==particlesB){
			n=i+1;
		}


		//B. Start the loop for all points of circleparticlesB
		for(;n<particlesB.size();n++){
			QParticle *pB=particlesB[n];
			if(pA==pB) continue;


			if(specifiedRadius==0.0f){
				radiusA=pA->GetRadius();
				radiusB=pB->GetRadius();
				
				totalRadius=radiusA+radiusB;
				totalRadiusPow=totalRadius*totalRadius;
			}


			//C. Check the distance between both points


			distVec=pB->GetGlobalPosition()-pA->GetGlobalPosition();
			positionalPenetrationSq=distVec.LengthSquared();

			

			//D. If the distance is less than radius of these points, create a new collision data
			if(positionalPenetrationSq<totalRadiusPow){
				//The penetration is the difference between the existing penetration and the total radius of the objA and the objB.
				positionalPenetration=sqrt(positionalPenetrationSq);
				normal=distVec.Normalized();

				penetration=totalRadius-positionalPenetration;

				contactPosition=pA->GetGlobalPosition()+radiusA*normal;

				//auto contact=QCollision::GetContactPool();
				QCollision::Contact *contact=QCollision::GetContactPool().Create().data;
				contact->Configure(pB,contactPosition,normal,penetration,vector<QParticle*>{pA});
				contacts.push_back(contact);
				

			}
		}
	}


}






void QCollision::CircleAndPolygon(vector<QParticle*> &circleParticles,vector<QParticle*> &polygonParticles,vector<QCollision::Contact*> &contacts){
	//The algorithm is an implement of the Separating Axis Theorem(SAT).
	/*
		A. Get a nearest points of polygonParticles
		B. Find the segments that contain nearest point.
		C. Make collision test with these segments
		D. If collision doesn't exist, make collision test with this nearest points
		E. If collission still doesn't exist, exit the function
	*/



	int polygonParticlesSize=polygonParticles.size();
	int circleParticlesSize=circleParticles.size();


	//A. Get a nearest points of polygonParticles
	for (int n=0;n<circleParticlesSize;n++){

		QParticle *circleParticle=circleParticles[n];
		// A. Get nearest point of polygonObject
		float minDistance=QWorld::MAX_WORLD_SIZE;
		//npi: nearest point index
		int npi=-1;
		QVector penetrationVec;
		float penetration=0.0f;
		QVector normal=QVector::Zero();
		for(int pi=0; pi<polygonParticlesSize; pi++){
			QParticle *p=polygonParticles[pi];
			QVector bridgeVec=circleParticle->GetGlobalPosition()-p->GetGlobalPosition();
			float dist=bridgeVec.Length();

			if(dist<minDistance){
				penetrationVec=bridgeVec;
				minDistance=dist;
				npi=pi;
				penetration=dist;
				normal=bridgeVec.Normalized();

			}
		}

		if(npi==-1)continue;


		//B. Find potantial segments that contain nearest point.
		//  np: nearest point

		int potantialSegmentIndexes[2][2]{ { ((npi-1)+polygonParticlesSize)%polygonParticlesSize, npi  }, // Segment Option A
									 { npi ,(npi+1)%polygonParticlesSize } }; // Segment Option B



		//C. Make the collision test with these segments and find reference segment
		minDistance=QWorld::MAX_WORLD_SIZE;
		bool seperationAxisDedected=false;
		int segmentIndex=-1;

		for(int i=0;i<2;i++){
			auto segmentOption=potantialSegmentIndexes[i];
			QParticle *sp1=polygonParticles[ segmentOption[0] ];
			QParticle *sp2=polygonParticles[ segmentOption[1] ];



			QVector segmentVector=sp2->GetGlobalPosition()-sp1->GetGlobalPosition();
			QVector segmentUnit=segmentVector.Normalized();
			QVector segmentNormal=segmentUnit.Perpendicular();

			QVector bridgeVec=circleParticle->GetGlobalPosition()-sp1->GetGlobalPosition();


			float projPerp=bridgeVec.Dot(segmentNormal);

			//If a seperation axis dedected, the collision doesn't exist with the circle point
			if(projPerp>=circleParticle->GetRadius()){
				seperationAxisDedected=true;
				break;
			}

			if(abs(projPerp)<minDistance && projPerp<circleParticle->GetRadius() ){
				float proj=bridgeVec.Dot(segmentUnit);
				if(proj>=0 && proj<=segmentVector.Length()){
					segmentIndex=i;
					minDistance=abs(projPerp);
					penetration=circleParticle->GetRadius()-projPerp;
					normal=segmentNormal;
					penetrationVec=penetration*normal;
				}
			}

		}



		if(seperationAxisDedected==true){
			continue;
		}

		//If we can find a reference segment
		if(segmentIndex!=-1){

			auto refSegmentIndexes=potantialSegmentIndexes[segmentIndex];

			QVector contactPosition=circleParticle->GetGlobalPosition();
			if(circleParticle->GetRadius()>0.5f){
				contactPosition-=circleParticle->GetRadius()*normal;
			}
			vector<QParticle*> refSegment={ polygonParticles[ refSegmentIndexes[0] ],polygonParticles[ refSegmentIndexes[1] ] };

			
			
			QCollision::Contact *contact=QCollision::GetContactPool().Create().data;
			contact->Configure(circleParticle,contactPosition,normal,penetration,refSegment );
			contacts.push_back(contact);

			continue;


		}


		
		//D. If collision doesn't exist, make collision test with this nearest point
		if(abs(penetration)<circleParticle->GetRadius()){
			penetration=circleParticle->GetRadius()-abs(penetration);
			QVector contactPosition=circleParticle->GetGlobalPosition();
			if(circleParticle->GetRadius()>0.5f){
				contactPosition-=circleParticle->GetRadius()*normal;
			}
			
			
			QCollision::Contact *contact=QCollision::GetContactPool().Create().data;
			contact->Configure(circleParticle,contactPosition,normal,penetration,vector<QParticle*>{ polygonParticles[npi] } );
			contacts.push_back(contact);

		}
	}



}

QCollision::~QCollision()
{
	
}

QObjectPool<QCollision::Contact> &QCollision::GetContactPool()
{
    return contactPool;
}

void QCollision::PolygonAndPolygon(vector<QParticle *> &particlesA, vector<QParticle *> &particlesB, vector<QCollision::Contact *> &contacts)
{
    //The algorithm is an implement of the Separating Axis Theorem(SAT).
	/*
		A. CHECK SEPERATING AXIS AND FIND  NORMAL IN MINIMUM PENETRATION
		B. FIND INCIDENT AND REFERENCE OBJECT/SEGMENT ACCORDING TO NORMAL
		C. CLIP POINTS AND DEFINE CONTACT POINTS
		D. RETURN COLLISION MANIFOLD
	*/

	int sizeparticlesA=particlesA.size();
	int sizeparticlesB=particlesB.size();

	int totalPointCount=sizeparticlesA+sizeparticlesB;

	vector<QParticle*> *refPolygon=&particlesA;
	vector<QParticle*> *incPolygon=&particlesB;
	int refPolygonSize=sizeparticlesA;

	float minPenetration=QWorld::MAX_WORLD_SIZE;
	QVector refNormal=QVector::Zero();
	//A. CHECK SEPERATING AXIS AND FIND  NORMAL IN MINIMUM PENETRATION
	int s=0;
	for(unsigned int p=0;p<totalPointCount;++p){
		if(p>=sizeparticlesA && refPolygon==&particlesA){
			refPolygon=&particlesB;
			refPolygonSize=sizeparticlesB;
			incPolygon=&particlesA;
			s=0;
		}
		QParticle *s1=(*refPolygon)[s];
		QParticle *s2=(*refPolygon)[(s+1)%refPolygonSize];

		QVector sNormal=(s2->GetGlobalPosition()-s1->GetGlobalPosition()).Normalized().Perpendicular();



		QCollision::Project refProject=ProjectToAxis(sNormal,*refPolygon);
		QCollision::Project incProject=ProjectToAxis(sNormal,*incPolygon);

		float penetration=refProject.Overlap(incProject);

		if(penetration>=0){
			return;
		}

		penetration=std::abs(penetration);

		if(penetration<minPenetration){
			minPenetration=penetration;
			refNormal=sNormal;
		}
		s+=1;

	}



	//B. FIND INCIDENT AND REFERENCE OBJECT/SEGMENT ACCORDING TO NORMAL
	QCollision::Project supportProjectA=ProjectToAxis(refNormal,particlesA);
	QCollision::Project supportProjectB=ProjectToAxis(refNormal,particlesB);

	int supportPointAIndex=supportProjectA.maxIndex;
	int supportPointBIndex=supportProjectB.minIndex;
	if(supportProjectB.min<supportProjectA.min){
		supportPointAIndex=supportProjectA.minIndex;
		supportPointBIndex=supportProjectB.maxIndex;
	}

	//particlesA Segment Option

	int segPointPrevA=( (supportPointAIndex-1)+sizeparticlesA )%sizeparticlesA;
	int segPointA=supportPointAIndex;
	int segPointNextA=(supportPointAIndex+1)%sizeparticlesA;

	QVector segmentAOption1=particlesA[segPointNextA]->GetGlobalPosition()-particlesA[segPointA]->GetGlobalPosition();
	QVector segmentAOption2=particlesA[segPointA]->GetGlobalPosition()-particlesA[segPointPrevA]->GetGlobalPosition();

	float segmentAOption1ParallelRate=std::abs( segmentAOption1.Dot(refNormal) );
	float segmentAOption2ParallelRate=std::abs( segmentAOption2.Dot(refNormal) );

	QParticle* segmentA[2]{particlesA[segPointA],particlesA[segPointNextA]};
	float segmentAParallelRate=segmentAOption1ParallelRate;
	if(segmentAOption2ParallelRate<segmentAOption1ParallelRate){
		segmentA[0]=particlesA[segPointPrevA];
		segmentA[1]=particlesA[segPointA];
		segmentAParallelRate=segmentAOption2ParallelRate;
	}

	//particlesB Segment Option

	int segPointPrevB=( (supportPointBIndex-1)+sizeparticlesB )%sizeparticlesB;
	int segPointB=supportPointBIndex;
	int segPointNextB=(supportPointBIndex+1)%sizeparticlesB;

	QVector segmentBOption1=particlesB[segPointNextB]->GetGlobalPosition()-particlesB[segPointB]->GetGlobalPosition();
	QVector segmentBOption2=particlesB[segPointB]->GetGlobalPosition()-particlesB[segPointPrevB]->GetGlobalPosition();

	float segmentBOption1ParallelRate=std::abs( segmentBOption1.Dot(refNormal) );
	float segmentBOption2ParallelRate=std::abs( segmentBOption2.Dot(refNormal) );

	QParticle* segmentB[2]{particlesB[segPointB],particlesB[segPointNextB] };
	float segmentBParallelRate=segmentBOption1ParallelRate;
	if(segmentBOption2ParallelRate<segmentBOption1ParallelRate){
		segmentB[0]=particlesB[segPointPrevB];
		segmentB[1]=particlesB[segPointB];
		segmentBParallelRate=segmentBOption2ParallelRate;
	}

	//Cliping and Adding contacts.
	if(segmentBParallelRate<segmentAParallelRate){
		//The reference segment is segmentB
		ClipContactParticles(segmentB,segmentA,contacts);
		if(contacts.size()==0)
			ClipContactParticles(segmentA,segmentB,contacts);
	}else{
		//The reference segment is segmentA
		ClipContactParticles(segmentA,segmentB,contacts);
		if(contacts.size()==0)
			ClipContactParticles(segmentB,segmentA,contacts);
	}
}

void QCollision::ClipContactParticles(QParticle *referenceParticles[], QParticle *incidentParticles[], vector<Contact*> &contacts)
{
	//segment vector
	auto sv=referenceParticles[1]->GetGlobalPosition()-referenceParticles[0]->GetGlobalPosition();
	auto len=sv.Length();
	auto unit=sv.Normalized();
	auto normal=unit.Perpendicular();
	for(unsigned int i=0;i<2;i++){
		QParticle* p=incidentParticles[i];
		QVector bv=p->GetGlobalPosition()-referenceParticles[0]->GetGlobalPosition();
		float dist=bv.Dot(normal);
		if(dist<=0){
			float proj=bv.Dot(unit);
			if(proj>=0.0f && proj<=len){
				QCollision::Contact *contact=QCollision::GetContactPool().Create().data;
				contact->Configure(p,p->GetGlobalPosition(),normal,abs(dist), vector<QParticle*>{referenceParticles[0],referenceParticles[1]});
				contacts.push_back(contact); 

			}
		}
	}



}


 QCollision::Project QCollision::ProjectToAxis(QVector &normal,vector<QParticle*> &polygon){
	float minDist=QWorld::MAX_WORLD_SIZE;
	float maxDist=-QWorld::MAX_WORLD_SIZE;
	int minPointIndex=0;
	int maxPointIndex=0;
	int polygonSize=polygon.size();
	for (unsigned int i=0;i<polygonSize;++i){
		float dist=polygon[i]->GetGlobalPosition().Dot(normal);
		if(dist<minDist){
			minDist=dist;
			minPointIndex=i;
		}
		if(dist>maxDist){
			maxDist=dist;
			maxPointIndex=i;
		}
	}



	return QCollision::Project(minDist,minPointIndex,maxDist,maxPointIndex);

 }

 pair<int, int> QCollision::FindNearestSideOfPolygon(const QVector point, vector<QParticle *> polygonParticles,bool checkSideRange)
 {
	

	pair<int,int> res(-1,-1);

	//A. Get a nearest points of polygonParticles
	int polygonSize=polygonParticles.size();
	// A. Get nearest point of polygonObject
	float minDistance=QWorld::MAX_WORLD_SIZE;
	

	
	for(int pi=0; pi<polygonSize; pi++){

		int npi=(pi+1)%polygonSize; //next particle index

		QParticle *p=polygonParticles[pi];
		QParticle *np=polygonParticles[ npi ];
		QVector bridgeVec=point-p->GetGlobalPosition();
		QVector sideVec=(np->GetGlobalPosition()-p->GetGlobalPosition());
		QVector sidePerp=sideVec.Perpendicular();

		if(checkSideRange){
			QVector sideUnit=sideVec.Normalized();
			float proj=bridgeVec.Dot( sideUnit );
			if( proj<0 || proj>sideVec.Length() ){
				continue;
			}	
		}

		float dist=bridgeVec.Dot( sidePerp );

		

		if(abs(dist)<minDistance ){
			res.first=pi;
			res.second=npi;
			minDistance=abs(dist);
		}
	}
	

     return res;
 }

 int QCollision::FindNearestParticleOfPolygon(QParticle* particle, vector<QParticle *> polygonParticles)
 {
	int res=0;
	float minDistance=QWorld::MAX_WORLD_SIZE;

	for(size_t i=0;i<polygonParticles.size();i++ ){
		QParticle * p=polygonParticles[i];
		if(p==particle)
			continue;
		float dist=(particle->GetGlobalPosition()-p->GetGlobalPosition()).Length();
		if(dist<minDistance){
			minDistance=dist;
			res=i;
		}
	}
	return res;
 }

 int QCollision::FindExtremeParticleOfAxis(vector<QParticle *> polygonParticles, QVector axisNormal)
 {
    int res=0;
	float maxDistance=-QWorld::MAX_WORLD_SIZE;

	for(size_t i=0;i<polygonParticles.size();i++ ){
		QParticle * p=polygonParticles[i];
		float proj=p->GetGlobalPosition().Dot(axisNormal);
		if(proj>maxDistance){
			maxDistance=proj;
			res=i;
		}
	}
	return res;
 }

 bool QCollision::PointInPolygonWN(const QVector point, vector<QParticle *> polygonParticles)
 {
	//Winding number algorithm for the point in polygon operations
     const QVector ray=QVector(QWorld::MAX_WORLD_SIZE,0.0f);
	 const QVector rayPerp=QVector::Down();

	 int windingNumber=0;

	size_t polygonSize=polygonParticles.size();

	 for (size_t i=0;i<polygonSize;++i ){
		QVector s1=polygonParticles[i]->GetGlobalPosition();
		QVector s2;
		if(i+1==polygonSize){
			s2=polygonParticles[0]->GetGlobalPosition();
		}else{
			s2=polygonParticles[ i+1 ]->GetGlobalPosition();
		}
		//Broadphase: Checking whether the point is in the range of y positions of the side
		if( point.y<=s1.y != point.y<=s2.y ){
			QVector sideVec=s2-s1;
			QVector sideVecPerp=sideVec.Perpendicular();
			QVector s1ToPoint=s1-point;
	
			float t=s1ToPoint.Dot( sideVecPerp )/ray.Dot(sideVecPerp );
			float u=(-s1ToPoint).Dot(rayPerp )/ sideVec.Dot(rayPerp);

			//Checking intersection between the ray and the side vector
			if( (t>=0.0 && t<=1.0) && (u>=0.0 && t<=1.0)  ){

				if(sideVec.y<0 )
					windingNumber-=1;
				else 
					windingNumber+=1;
			}

		}
	 }

	 return windingNumber!=0;
 }

 QVector QCollision::LineIntersectionLine(QVector d1A, QVector d1B, QVector d2A, QVector d2B)
 {
	 QVector v1=d1B-d1A;

	 QVector v2=d2B-d2A;


	 QVector v2Normal=v2.Perpendicular();

	 QVector vb1=d2A-d1A;

	 float t= vb1.Dot( v2Normal ) / v1.Dot( v2Normal) ;
	 if(t<0 || t>1 ||  t==0){
		 return QVector::NaN();
	 }

	 QVector v1Normal=v1.Perpendicular();

	 float u= -vb1.Dot(v1Normal) / v2.Dot (v1Normal);

	 if(u<0 || u>1){
		 return QVector::NaN();
	 }

	 QVector result=d1A+t*v1;


	 return result;
 }

 bool QCollision::PointInPolygon(QVector &point, vector<QParticle*> &polygon)
 {
	 int polygonSize=polygon.size();
	 for(int i=0;i<polygonSize;i++){
		 QParticle *p=polygon[i];
		 QParticle *np=polygon[ (i+1)%polygonSize ];
		 QVector normal=(np->GetGlobalPosition()-p->GetGlobalPosition()).Normalized().Perpendicular();
		 float perpProj=(point-p->GetGlobalPosition()).Dot(normal);
		 if(perpProj>0)return false;
	 }
	 return true;
 }

 bool QCollision::PointInPolygon(QVector &point, vector<QVector> &polygon)
 {
	 vector<QParticle*> particles;
	 for(int i=0;i<polygon.size();i++){
		 particles.push_back( new QParticle(polygon[i] ) );
	 }
	 bool res=PointInPolygon(point,particles);

	 for(int i=0;i<particles.size();i++){
		 delete particles[i];
	 }

	 return res;
 }

 bool QCollision::PointInPolygon2(QVector point, vector<QParticle *> &polygon)
 {

	   int i, j, nvert = polygon.size();
	   bool c = false;

	   for(i = 0, j = nvert - 1; i < nvert; j = i++) {
		 if( ( (point.y < polygon[i]->GetGlobalPosition().y  ) != ( point.y< polygon[j]->GetGlobalPosition().y) ) &&
			 (point.x < (polygon[j]->GetGlobalPosition().x - polygon[i]->GetGlobalPosition().x) * (point.y - polygon[i]->GetGlobalPosition().y) / (polygon[j]->GetGlobalPosition().y - polygon[i]->GetGlobalPosition().y) + polygon[i]->GetGlobalPosition().x)
		   )
		   c = !c;
	   }

	   return c;
 }


 