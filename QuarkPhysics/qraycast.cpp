
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

#include "qraycast.h"
#include "qworld.h"
#include <algorithm>


QRaycast::QRaycast(QVector position, QVector rayVector, bool enableContainingBodies)
{
	this->position=position;
	this->ray=rayVector;
	this->rayOriginal=rayVector;
	this->enabledContainingBodies=enableContainingBodies;
}

vector<QBody *> QRaycast::GetPotentialBodies(QWorld *whichWorld, QVector rayPosition, QVector rayVector, int collidableLayers)
{
	vector<QBody*> res;

	QVector rayEndPosition=rayPosition+rayVector;

	bool isRayToNegativeX=rayVector.x<0;
	bool isRayToNegativeY=rayVector.y<0;

	for(int i=0;i<whichWorld->GetBodyCount();i++){
		QBody* body=whichWorld->GetBodyAt(i);
		if(isRayToNegativeX){
			if(body->GetAABB().GetMin().x>rayPosition.x || body->GetAABB().GetMax().x<rayEndPosition.x){
				continue;
			}
		}else{
			if(body->GetAABB().GetMax().x<rayPosition.x || body->GetAABB().GetMin().x>rayEndPosition.x){
				continue;
			}
		}

		if(isRayToNegativeY){
			if(body->GetAABB().GetMin().y>rayPosition.y || body->GetAABB().GetMax().y<rayEndPosition.y){
				continue;
			}
		}else{
			if(body->GetAABB().GetMax().y<rayPosition.y || body->GetAABB().GetMin().y>rayEndPosition.y){
				continue;
			}
		}
		res.push_back(body);
	}
	return res;
}



void QRaycast::UpdateContacts()
{
	if(world==nullptr)return;
	contacts=RaycastTo(world,position,ray,1,enabledContainingBodies);
}



vector<QRaycast::Contact> QRaycast::RaycastTo(QWorld *world, QVector rayPosition, QVector rayVector, int collidableLayers,bool enableContainingBodies)
{
	vector<QRaycast::Contact> result;
	vector<QBody*> potantialBodies=GetPotentialBodies(world,rayPosition,rayVector,collidableLayers);

	QVector rayUnit=rayVector.Normalized();
	QVector rayNormal=rayUnit.Perpendicular();

	for(auto body:potantialBodies){
		for(int i=0;i<body->GetMeshCount();i++){
			QMesh *mesh=body->GetMeshAt(i);
			if(mesh->GetCollisionBehavior()==QMesh::CollisionBehaviors::CIRCLES){
				RaycastToParticles(body,mesh,rayPosition,rayVector,rayUnit,rayNormal,enableContainingBodies,&result);
			}else if(mesh->GetCollisionBehavior()==QMesh::CollisionBehaviors::POLYGONS){
				RaycastToPolygon(body,mesh,rayPosition,rayVector,rayUnit,rayNormal,enableContainingBodies,&result);
			}
		}
	}

	std::sort(result.begin(),result.end(),SortContacts);
	return result;

}

vector<QRaycast::Contact> *QRaycast::GetContacts()
{
	return &contacts;
}

QVector QRaycast::GetPosition()
{
	return position;
}

QVector QRaycast::GetRayVector()
{
	return ray;
}

bool QRaycast::GetEnabledContainingBodies()
{
	return enabledContainingBodies;
}



float QRaycast::GetRotation()
{
	return rotation;
}

QRaycast *QRaycast::SetPosition(QVector value)
{
	position=value;
	return this;
}

QRaycast *QRaycast::SetRotation(float value)
{
	rotation=value;
	ray=rayOriginal.Rotated(rotation);
	return this;
}

QRaycast *QRaycast::SetRayVector(QVector value)
{
	rayOriginal=value;
	ray=rayOriginal.Rotated(rotation);
	return this;
}

QRaycast *QRaycast::SetEnabledContainingBodies(bool value)
{
	enabledContainingBodies=value;
	return this;
}

void QRaycast::RaycastToParticles(QBody *body, QMesh *mesh, QVector rayPosition, QVector rayVector, QVector rayUnit, QVector rayNormal, bool enableContainingBodies,vector<QRaycast::Contact> *contacts)
{
	int nearParticleIndex=-1;
	float nearDistance=QWorld::MAX_WORLD_SIZE;
	QVector nearContactPosition=QVector::Zero();
	QVector nearContactNormal=QVector::Zero();

	for(int i=0;i<mesh->GetParticleCount();i++){
		QParticle * p=mesh->GetParticle(i);
		QVector bridge=p->GetGlobalPosition()-rayPosition;
		float proj=bridge.Dot(rayUnit);
		if((proj-p->GetRadius())>nearDistance)continue;
		float perpProj=bridge.Dot(rayNormal);
		if(abs(perpProj)<p->GetRadius() && (proj>=-p->GetRadius() && proj<=rayVector.Length()+p->GetRadius()) ){
			QVector projPosOnRay=rayPosition+proj*rayUnit;
			float hipotenus=sqrt(p->GetRadius()*p->GetRadius()-perpProj*perpProj);
			QVector contactPosition=projPosOnRay-(hipotenus*rayUnit);
			float nProj=(contactPosition-rayPosition).Dot(rayUnit);

			if(enableContainingBodies){
				if(nProj<=0){
					contactPosition=rayPosition;
				}
			}else{
				if(nProj<=0){
					continue;
				}
			}

			nearDistance=proj;
			nearParticleIndex=i;
			nearContactPosition=contactPosition;
			nearContactNormal=(contactPosition-p->GetGlobalPosition()).Normalized();

		}
	}
	if(nearParticleIndex==-1)
		return;
	QRaycast::Contact contact(body,nearContactPosition,nearContactNormal,nearDistance);
	contacts->push_back(contact);

}

void QRaycast::RaycastToPolygon(QBody *body, QMesh *mesh, QVector rayPosition, QVector rayVector, QVector rayUnit, QVector rayNormal, bool enableContainingBodies,vector<QRaycast::Contact> *contacts)
{
	QVector rayStart=rayPosition;
	QVector rayEnd=rayPosition+rayVector;


	bool contactFound=false;
	float nearDistance=QWorld::MAX_WORLD_SIZE;
	QVector nearContactPosition=QVector::Zero();
	QVector nearContactNormal=QVector::Zero();


	for(int n=0;n<mesh->GetClosedPolygonCount();n++){
		vector<QParticle*> &polygon=mesh->GetClosedPolygon(n);
		for(int i=0;i<polygon.size();i++){

			QParticle *p=polygon[i];
			QParticle *np=polygon[(i+1)%polygon.size() ];

			QVector intersection=QCollision::LineIntersectionLine(p->GetGlobalPosition(),np->GetGlobalPosition(),rayStart,rayEnd);
			if(intersection.isNaN())continue;

			float distance=(intersection-rayStart).Length();
			if(distance>nearDistance)continue;

			nearDistance=distance;
			nearContactPosition=intersection;
			nearContactNormal=(np->GetGlobalPosition()-p->GetGlobalPosition()).Normalized().Perpendicular();
			contactFound=true;




		}
	}

	if(contactFound==false)return;

	if(rayVector.Dot(nearContactNormal)>0){
		//It's a containing body
		if(enableContainingBodies){
			nearContactPosition=rayPosition;

		}else{
			return;
		}
	}

	QRaycast::Contact contact(body,nearContactPosition,nearContactNormal,nearDistance);
	contacts->push_back(contact);
}

bool QRaycast::SortContacts(Contact contactA, Contact contactB)
{
	return contactA.distance<contactB.distance;
}
