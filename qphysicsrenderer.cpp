
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

#include "qphysicsrenderer.h"
#include "QuarkPhysics/qareabody.h"
#include "QuarkPhysics/qrigidbody.h"

QPhysicsRenderer::QPhysicsRenderer()
{

}

void QPhysicsRenderer::RenderSpatialCells(QWorld *world, sf::RenderWindow *window)
{
//	auto &brc=world->broadPhase.collisionGroups;
//	auto &cellSize=world->broadPhase.cellSize;
//	sf::Color col=sf::Color::White;
//	col.a=15.0f;
//	for(int y=0;y!=20;y++){
//		for(int x=0;x!=20;x++){
//			auto pos=sf::Vector2f(x*cellSize.x,y*cellSize.y);
//			sf::RectangleShape shape(sf::Vector2f(cellSize.x,cellSize.y));

//			shape.setOutlineColor(col);
//			shape.setOutlineThickness(1.0f);
//			shape.setFillColor(sf::Color::Transparent);

//			shape.setPosition(pos);
//			window->draw(shape);
//		}
//	}
}

void QPhysicsRenderer::RenderBoundingBoxes(QWorld *world, sf::RenderWindow *window)
{
	for(int i=0;i<world->GetBodyCount();i++){
		QAABB aabb=world->GetBodyAt(i)->GetAABB();
		sf::Vector2f size(aabb.GetSize().x,aabb.GetSize().y);
		sf::RectangleShape shape(size);
		shape.setFillColor(sf::Color::Transparent);
		shape.setOutlineColor(sf::Color::White);
		shape.setOutlineThickness(1.0f);
		auto rectCenter=aabb.GetCenterPosition();
		shape.setPosition(rectCenter.x-size.x/2.0f,rectCenter.y-size.y/2.0f);
		window->draw(shape);
	}
}

void QPhysicsRenderer::RenderColliders(QWorld *world, sf::RenderWindow *window)
{
	for(int n=0;n<world->GetBodyCount();n++){
		QBody * body=world->GetBodyAt(n);
		sf::Color col=QPhysicsRenderer::COLOR_BODY_DYNAMIC;
		if(body->GetIsSleeping()  ){
			col=QPhysicsRenderer::COLOR_BODY_DYNAMIC_SLEEPING;
		}
		col=body->GetMode()==QBody::STATIC ? QPhysicsRenderer::COLOR_BODY_STATIC:col;

		
		if(body->GetBodyType()==QBody::BodyTypes::RIGID){
			QRigidBody *rbody=static_cast<QRigidBody*>(body);
			col=rbody->GetKinematicEnabled()==true ? QPhysicsRenderer::COLOR_BODY_DYNAMIC_KINEMATIC:col;
		}
		
		if(body->GetBodyType()==QBody::BodyTypes::AREA){
			col=QPhysicsRenderer::COLOR_AREA;
		}
			

		for(int i=0;i<body->GetMeshCount();i++){
			QMesh *mesh=body->GetMeshAt(i);
			if(mesh->GetCollisionBehavior()==QMesh::CIRCLES){
				for(int n=0;n<mesh->GetParticleCount();n++){
					QParticle *particle=mesh->GetParticle(n);
					float r=particle->GetRadius();
					sf::CircleShape renderShape(r);
					renderShape.setOutlineColor(col );
					renderShape.setOutlineThickness(1.0f);
					renderShape.setFillColor(sf::Color::Transparent);
					renderShape.setPosition(particle->GetGlobalPosition().x-r,particle->GetGlobalPosition().y-r);
					window->draw(renderShape);
					auto rotVec=QVector::AngleToUnitVector(body->GetRotation())*r;
					sf::Vertex circleLine[]{
						sf::Vertex(sf::Vector2f(particle->GetGlobalPosition().x,particle->GetGlobalPosition().y),col ),
						sf::Vertex(sf::Vector2f(particle->GetGlobalPosition().x+rotVec.x,particle->GetGlobalPosition().y+rotVec.y),col )
					};
					window->draw(circleLine,2,sf::Lines);
				}

			}else{
				int particleSize=mesh->GetParticleCount();
				if(body->GetSimulationModel()!=QBody::SimulationModels::RIGID_BODY){
					//Draw Particles
					for(int p=0;p<particleSize;p++){
						QParticle *particle=mesh->GetParticle(p);
						if(particle->GetRadius()>0.5f){
							float radius=particle->GetRadius();
							sf::CircleShape particleShape(radius );
							particleShape.setOutlineColor(col);
							particleShape.setOutlineThickness(1.0f);
							particleShape.setFillColor(sf::Color::Transparent);
							particleShape.setPosition(particle->GetGlobalPosition().x-radius,particle->GetGlobalPosition().y-radius);
							window->draw(particleShape);
						}

					}
					//Draw Springs
					QSoftBody *sBody=static_cast<QSoftBody*>(body);
					int springSize=mesh->GetSpringCount();
					for(int s=0;s<springSize;s++){
						QSpring *spring=mesh->GetSpringAt(s);
						//if(spring->GetIsInternal()==false)continue;
						//if(sBody->GetVolumePreserving()==true && spring->GetIsInternal()==true)continue;
						sf::Color springCol=spring->GetIsInternal() ? QPhysicsRenderer::COLOR_SPRING_INTERNAL:QPhysicsRenderer::COLOR_SPRING;
						QVector sAPos=spring->GetParticleA()->GetGlobalPosition();
						QVector sBPos=spring->GetParticleB()->GetGlobalPosition();

						sf::Vertex springLine[]{
							sf::Vertex(sf::Vector2f(sAPos.x,sAPos.y),springCol ),
							sf::Vertex(sf::Vector2f(sBPos.x,sBPos.y),springCol ),
						};
						window->draw(springLine,2,sf::Lines);
					}
				}
				//Draw Polygons
				for(int p=0;p<mesh->GetClosedPolygonCount();p++){
					vector<QParticle*> *polygon=&mesh->GetClosedPolygon(p);
					sf::ConvexShape polygonShape(polygon->size());

					vector<sf::Vertex> vertices;

					for(int t=0;t<polygon->size();t++){
						QParticle *particle=polygon->at(t);
						QVector particlePos=particle->GetGlobalPosition();
						vertices.push_back(sf::Vertex(sf::Vector2f(particlePos.x,particlePos.y),col ) );
					}
					vertices.push_back( vertices[0]);
					window->draw(&vertices[0],vertices.size(),sf::LineStrip);
				}




			}
		}

	}
}

void QPhysicsRenderer::RenderJointsAndSprings(QWorld *world, sf::RenderWindow *window)
{
	sf::Color col=QPhysicsRenderer::COLOR_JOINT;

	float circleSize=2;
	sf::Vector2f circleSizeVec(circleSize,circleSize);
	int circleSides=12;

	for(int i=0;i<world->GetJointCount();i++){
		QJoint *joint=world->GetJointAt(i);

		sf::Vector2f vA(joint->GetAnchorAGlobalPosition().x,joint->GetAnchorAGlobalPosition().y);
		sf::Vector2f vB(joint->GetAnchorBGlobalPosition().x,joint->GetAnchorBGlobalPosition().y);

		sf::CircleShape csA( circleSize,circleSides );
		csA.setPosition(vA-circleSizeVec );
		csA.setFillColor(col);
		window->draw(csA);

		sf::CircleShape csB( circleSize,circleSides );
		csB.setPosition(vB-circleSizeVec);
		csB.setFillColor(col);
		window->draw(csB);

		sf::Vertex jointLine[]{
			sf::Vertex(vA,col ),
			sf::Vertex(vB,col )
		};
		window->draw(jointLine,2,sf::Lines);
	}

	col=QPhysicsRenderer::COLOR_SPRING;
	for(int i=0;i<world->GetSpringCount();i++){
		QSpring *spring=world->GetSpringAt(i);

		sf::Vector2f vA(spring->GetParticleA()->GetGlobalPosition().x,spring->GetParticleA()->GetGlobalPosition().y);
		sf::Vector2f vB(spring->GetParticleB()->GetGlobalPosition().x,spring->GetParticleB()->GetGlobalPosition().y);

		sf::CircleShape csA( circleSize,circleSides );
		csA.setPosition(vA-circleSizeVec);
		csA.setFillColor(col);
		window->draw(csA);

		sf::CircleShape csB( circleSize,circleSides );
		csB.setPosition(vB-circleSizeVec);
		csB.setFillColor(col);
		window->draw(csB);

		sf::Vertex jointLine[]{
			sf::Vertex(vA,col ),
			sf::Vertex(vB,col )
		};
		window->draw(jointLine,2,sf::Lines);
	}
}

void QPhysicsRenderer::RenderRaycasts(QWorld *world, sf::RenderWindow *window)
{
	auto col=QPhysicsRenderer::COLOR_RAYCAST;
	float pointRadius=3.0f;
	for(int i=0;i<world->GetRaycastCount();i++){
		QRaycast *raycast=world->GetRaycastAt(i);
		QVector rayStart=raycast->GetPosition();
		QVector rayEnd=rayStart+raycast->GetRayVector();
		if(raycast->GetContacts()->size()>0){
			rayEnd=raycast->GetContacts()->at(0).position;
			sf::CircleShape contactPointShape(pointRadius);
			contactPointShape.setFillColor(col );
			contactPointShape.setPosition(sf::Vector2f(rayEnd.x-pointRadius,rayEnd.y-pointRadius));
			window->draw(contactPointShape);
		}
		sf::Vertex line[]{
			sf::Vertex( sf::Vector2f(rayStart.x,rayStart.y) ,col),
			sf::Vertex(sf::Vector2f(rayEnd.x,rayEnd.y),col )
		};
		window->draw(line,2,sf::Lines);
	}
}

void QPhysicsRenderer::RenderPhysicsGizmos(QWorld *world, sf::RenderWindow *window)
{
	for(int i=0;i<world->GetGizmos()->size();i++){

		QGizmo *gizmo=world->GetGizmos()->at(i);
		auto col=COLOR_CONTACT;
		if(gizmo->GetGizmoType()==QGizmo::Circle){
			QGizmoCircle *gizmoCircle=static_cast<QGizmoCircle*>(gizmo);
			float r=gizmoCircle->radius;
			sf::CircleShape renderShape(r);
			renderShape.setFillColor(col );
			renderShape.setPosition(sf::Vector2f(gizmoCircle->position.x-r,gizmoCircle->position.y-r) );
			window->draw(renderShape);

		}else if(gizmo->GetGizmoType()==QGizmo::Line){
			QGizmoLine *gizmoLine=static_cast<QGizmoLine*>(gizmo);
			sf::Vertex line[]{
				sf::Vertex(sf::Vector2f(gizmoLine->pointA.x,gizmoLine->pointA.y) ),
				sf::Vertex(sf::Vector2f(gizmoLine->pointB.x,gizmoLine->pointB.y) )
			};
			line[0].color=col;
			line[1].color=col;
			window->draw(line,2,sf::Lines);

			if(gizmoLine->isArrow){
				QVector lineVec=gizmoLine->pointB-gizmoLine->pointA;
				QVector unit=lineVec.Normalized();
				float arrowLen=lineVec.Length()*0.1f;
				QVector arrowA=gizmoLine->pointB-(unit*arrowLen+unit.Perpendicular()*arrowLen);
				QVector arrowB=gizmoLine->pointB-(unit*arrowLen-unit.Perpendicular()*arrowLen);
				sf::Vertex lineA[]{
					sf::Vertex(sf::Vector2f(gizmoLine->pointB.x,gizmoLine->pointB.y) ),
					sf::Vertex(sf::Vector2f(arrowA.x,arrowA.y) )
				};
				sf::Vertex lineB[]{
					sf::Vertex(sf::Vector2f(gizmoLine->pointB.x,gizmoLine->pointB.y) ),
					sf::Vertex(sf::Vector2f(arrowB.x,arrowB.y) )
				};
				lineA->color=col;
				lineB->color=col;
				window->draw(lineA,2,sf::Lines);
				window->draw(lineB,2,sf::Lines);
			}
		}
	}
}
