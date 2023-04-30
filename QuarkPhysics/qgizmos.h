#ifndef QGIZMOS_H
#define QGIZMOS_H
#include "qvector.h"

class QGizmo
{

	public:
		QGizmo(){

		};
		virtual ~QGizmo(){}
};
class QGizmoCircle:public QGizmo{
	public:
		float radius=0.0f;
		QVector position=QVector::Zero();
		QGizmoCircle(QVector pos,float rad ){
			position=pos;
			radius=rad;
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
		}
};



#endif // QGIZMOS_H
