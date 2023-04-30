#ifndef QAREABODY_H
#define QAREABODY_H
#include "qbody.h"
#include "qworld.h"
#include <functional>
#include <unordered_set>

class QManifold;
/**
 * @brief QAreaBody objects are objects that don't respond to collisions or receive any response from them, but only report collisions. An operation is not applied for them to move during physics steps, they are stationary. Unlike other body types, they have two event listeners named OnCollisionEnter and OnCollisionExit.
 */
class QAreaBody : public QBody{
	unordered_set<QBody*> bodies;
	void AddCollidedBody(QBody *body);
	void CheckBodies();
public:
	QAreaBody();
	/** This event listener is triggered when any object enters the collision list of a QAreaBody for the first time.
	 * @param collidedBody The body entered to the collision list.
	 */
	virtual void OnCollisionEnter(QBody *collidedBody){};
	/** This event listener is triggered when any object exits the collision list of a QAreaBody for the first time.
	 * @param collidedBody The body exited from the collision list.
	 */
	virtual void OnCollisionExit(QBody *collidedBody){};
	/** This is the event listener callback function for the OnCollisionEnter event.
	 * @param areaBody An area body.
	 * @param collidedBody The collided body.
	 */
	std::function<void(QAreaBody *areaBody,QBody* collidedBody)> CollisionEnterEventListener;
	/** This is the event listener callback function for the OnCollisionExit event.
	 * @param areaBody An area body.
	 * @param collidedBody The collided body.
	 */
	std::function<void(QAreaBody *areaBody,QBody* collidedBody)> CollisionExitEventListener;


	friend class QManifold;
	friend class QWorld;

};


#endif //QAREABODY_H
