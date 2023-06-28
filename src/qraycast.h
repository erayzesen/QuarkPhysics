#ifndef QRAYCAST_H
#define QRAYCAST_H
#include "qvector.h"
#include "qbody.h"

class QWorld;

class QRaycast
{

protected:
	void UpdateContacts();

public:
	QRaycast(QVector position, QVector rayVector,bool enableContainingBodies=false);

	struct Contact{
	public:
		QBody *body;
		QVector position;
		QVector normal;
		float distance;
		Contact(QBody *body, QVector position, QVector normal,float distance): body(body), position(position),normal(normal),distance(distance){}

	};

	static vector<QRaycast::Contact> RaycastTo(QWorld *world, QVector rayPosition,QVector rayVector, int collidableLayers=1,bool enableContainingBodies=false );





	//Get Methods
	vector<QRaycast::Contact> *GetContacts();
	QVector GetPosition();
	float GetRotation();
	QVector GetRayVector();
	bool GetEnabledContainingBodies();

	//Set Methods
	QRaycast *SetPosition(QVector val);
	QRaycast *SetRotation(float val);
	QRaycast *SetRayVector(QVector val);
	QRaycast *SetEnabledContainingBodies(bool val);


	friend class QWorld;

private:
	QVector position=QVector::Zero();
	float rotation=0.0f;
	QVector ray=QVector::Zero();
	QVector rayOriginal=QVector::Zero();

	bool enabledContainingBodies=false;

	vector<QRaycast::Contact> contacts;

	QWorld *world;



	static vector<QBody*> GetPotentialBodies(QWorld *whichWorld,QVector rayPosition,QVector rayVector,int collidableLayers);
	static void RaycastToParticles(QBody *body, QMesh *mesh, QVector rayPosition, QVector rayVector, QVector rayUnit,QVector rayNormal, bool enableContainingBodies,vector<QRaycast::Contact> *contacts);
	static void RaycastToPolygon(QBody *body, QMesh *mesh, QVector rayPosition, QVector rayVector, QVector rayUnit,QVector rayNormal, bool enableContainingBodies,vector<QRaycast::Contact> *contacts);

	static bool SortContacts(const QRaycast::Contact contactA,const QRaycast::Contact contactB);

};

#endif // QRAYCAST_H
