
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

#ifndef QGIZMOS_H
#define QGIZMOS_H
#include "qvector.h"
#include "qaabb.h"

class QGizmo
{
	public :
		enum GizmoTypes{
			Circle,
			Line,
			Rectangle
		};
	protected:
		GizmoTypes gizmoType=GizmoTypes::Circle;
	public:
		
		QGizmo(){

		};
		virtual ~QGizmo(){}
		
		GizmoTypes GetGizmoType(){
			return gizmoType;
		}
};
class QGizmoCircle:public QGizmo{
	public:
		float radius=0.0f;
		QVector position=QVector::Zero();
		QGizmoCircle(QVector pos,float rad ){
			position=pos;
			radius=rad;
			gizmoType=GizmoTypes::Circle;
		}
};

class QGizmoLine:public QGizmo{
	public:
		QVector pointA=QVector::Zero();
		QVector pointB=QVector::Zero();
		bool isArrow;
		QGizmoLine(QVector from,QVector to,bool arrow=false ){
			pointA=from;
			pointB=to;
			isArrow=arrow;
			gizmoType=GizmoTypes::Line;
		}
};

class QGizmoRect:public QGizmo{
	public:
		QAABB rect;
		
		QGizmoRect( QAABB rectangle ){
			rect=rectangle;
			gizmoType=GizmoTypes::Rectangle;
		}
};



#endif // QGIZMOS_H
