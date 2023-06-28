
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





void QCollision::PolylineAndCircle(vector<QParticle*> &polylineParticles, vector<QParticle*> &circleParticles, vector<Contact> &contacts)
{
	CircleAndCircle(polylineParticles,circleParticles,contacts);

	/*
	A. Start the loop for all segments of polyline
	B. Check the perpandicular distance between the circle position and the segment position
	C. Check If the circle object is in the segment range.
	D. If the segment distance of the circle position is smaller than the radius and the circleObj position is in the range of the segment, apply collision.
	*/


	for(int n=0;n<circleParticles.size();n++){
		QParticle *cp=circleParticles[n];
		for(int i=0;i<polylineParticles.size();i++){

			QParticle *s1=polylineParticles[i];
			QParticle *s2=polylineParticles[ (i+1)%polylineParticles.size() ];

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



			QVector bridgeVec=cp->GetGlobalPosition()-s1Pos;

			QVector testVirtualSegment[2];

			testVirtualSegment[0]=s1Pos;
			testVirtualSegment[1]=s2Pos;


			QVector testBridgeVec=cp->GetGlobalPosition()-testVirtualSegment[0];

			float perpProj=testBridgeVec.Dot(normal);

			if(abs(perpProj)<cp->GetRadius()){
				float proj=bridgeVec.Dot(unit);
				if(proj>=0 && proj<=len){
					int projSign=perpProj<0 ? -1:1;
					float penetration=abs( (cp->GetRadius()*projSign)-perpProj);
					QVector contactPosition=cp->GetGlobalPosition()-(cp->GetRadius()*projSign*normal);

					QCollision::Contact contact(cp,contactPosition,normal,penetration,vector<QParticle*>{s1,s2});

					contacts.push_back(contact);

				}
			}



		}
	}


}



void QCollision::PolylineAndPolygon(vector<QParticle*> &polylineParticles, vector<QParticle*> &polygonParticles, vector<Contact> &contacts)
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


		QVector prevToNextVec=np->GetGlobalPosition()-pp->GetGlobalPosition();
		QVector bridgeVec=p->GetGlobalPosition()-pp->GetGlobalPosition();
		QVector prevToNextPerpVec=prevToNextVec.Perpendicular();


		//We don't want to check from concave corners
		/*if(bridgeVec.Dot(prevToNextPerpVec)<0){
			bisectorList.push_back(QVector::Zero() );
			continue;
		}*/

		QVector bisectorUnit=prevToNextPerpVec.Normalized();
		
		QVector bisectorRay=-bisectorUnit*QWorld::MAX_WORLD_SIZE;

		

		int sia=ni; // segment index a
		QVector bisectorVector=QVector::Zero();
		if(polylineParticles==polygonParticles){
			float rayLength=abs(bridgeVec.Dot(prevToNextPerpVec.Normalized() ) )*0.5f;
			
			bisectorVector=-bisectorUnit*rayLength;
		}else{
			
			float minDistance=QWorld::MAX_WORLD_SIZE;
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
				}
				sia=sib;
			}
		}


		bisectorList.push_back(bisectorVector );
		p->GetOwnerMesh()->GetOwnerBody()->GetWorld()->gizmos.push_back(new QGizmoLine(p->GetGlobalPosition(),p->GetGlobalPosition()+bisectorVector,true) );

		/*
		Notes: Bisector vector is working well now but it doesn't looks true. A different method is followed here.  
		*/



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


			QCollision::Contact contact(p,p->GetGlobalPosition(),normal,penetration,vector<QParticle*>{s1,s2});

			contacts.push_back(contact);

		}
	}

}


void QCollision::CircleAndCircle(vector<QParticle*> &particlesA,vector<QParticle*> &particlesB,vector<QCollision::Contact> &contacts){

	/*
	A. Start the loop for all points of particlesA
	B. Start the loop for all points of particlesB
	C. Check the distance between both points
	D. If the distance is less than radius of these points, apply the collision
	*/




	//A. Start the loop for all points of circleparticlesA
	for(int i=0;i<particlesA.size();i++){
		QParticle *pA=particlesA[i];

		//B. Start the loop for all points of circleparticlesB
		for(int n=0;n<particlesB.size();n++){
			QParticle *pB=particlesB[n];
			if(pA==pB) continue;

			//C. Check the distance between both points


			QVector distVec=pB->GetGlobalPosition()-pA->GetGlobalPosition();
			QVector normal=distVec.Normalized();
			float positionalPenetration=distVec.Length();

			float radiusA=pA->GetRadius();
			float radiusB=pB->GetRadius();

			//D. If the distance is less than radius of these points, create a new collision data
			if(positionalPenetration<radiusA+radiusB){
				//The penetration is the difference between the existing penetration and the total radius of the objA and the objB.

				float penetration=(radiusA+radiusB)-positionalPenetration;

				QVector contactPosition=pA->GetGlobalPosition()+radiusA*normal;

				QCollision::Contact contact(pB,contactPosition,normal,penetration,vector<QParticle*>{pA});
				contacts.push_back(contact);

			}
		}
	}


}






void QCollision::CircleAndPolygon(vector<QParticle*> &circleParticles,vector<QParticle*> &polygonParticles,vector<QCollision::Contact> &contacts){
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

			QCollision::Contact contact(circleParticle,contactPosition,normal,penetration,refSegment );


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
			QCollision::Contact contact(circleParticle,contactPosition,normal,penetration,vector<QParticle*>{ polygonParticles[npi] } );


			contacts.push_back(contact);

		}
	}



}



void QCollision::PolygonAndPolygon(vector<QParticle*> &particlesA,vector<QParticle*> &particlesB, vector<QCollision::Contact> &contacts){
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

void QCollision::ClipContactParticles(QParticle *referenceParticles[], QParticle *incidentParticles[], vector<Contact> &contacts)
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
				Contact c(p,p->GetGlobalPosition(),normal,abs(dist), vector<QParticle*>{referenceParticles[0],referenceParticles[1]});
				contacts.push_back(c);

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

 QVector QCollision::LineIntersectionLine(QVector d1A, QVector d1B, QVector d2A, QVector d2B)
 {
	 QVector v1=d1B-d1A;

	 QVector v2=d2B-d2A;


	 QVector v1Normal=v1.Perpendicular();

	 QVector v2Normal=v2.Perpendicular();

	 QVector vb1=d2A-d1A;

	 float t= vb1.Dot( v2Normal ) / v1.Dot( v2Normal) ;
	 float u= -vb1.Dot(v1Normal) / v2.Dot (v1Normal);

	 if(t<0 || t>1 || u<0 || u>1 || t==0){
		 return QVector::NaN();
	 }

	 QVector result2=d1A+t*v1;


	 return result2;
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
		 if( ( (polygon[i]->GetGlobalPosition().y >= point.y ) != (polygon[j]->GetGlobalPosition().y >= point.y) ) &&
			 (point.x <= (polygon[j]->GetGlobalPosition().x - polygon[i]->GetGlobalPosition().x) * (point.y - polygon[i]->GetGlobalPosition().y) / (polygon[j]->GetGlobalPosition().y - polygon[i]->GetGlobalPosition().y) + polygon[i]->GetGlobalPosition().x)
		   )
		   c = !c;
	   }

	   return c;
 }



