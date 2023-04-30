#ifndef QJOINT_H
#define QJOINT_H
#include "QRigidBody.h"


/**
 * @brief QJoint objects serves to apply various distance constraints between rigid bodies. Additionally, you can create a distance constraint between any object and an imaginary point in space. Instead of separate methods for all fundamental constraints in physics engines, there is a set of property sets available. For example, setting a distance constraint to a distance of 0 creates a constraint known as a pin joint. By decreasing the rigidity of a set distance constraint, you obtain another type of joint called a spring joint. Enabling the groove mode prevents the constraint from being applied as long as the set distance is not exceeded, resulting in another type of joint called a groove joint. QWorld also provides methods to manage QJoint objects.
 */
class QJoint
{
	//Base Properties

	QRigidBody * bodyA=nullptr;
	QRigidBody * bodyB=nullptr;

	QVector anchorA;
	QVector anchorB;

	QVector anchorGlobalA;
	QVector anchorGlobalB;

	bool collisionsEnabled=false;

	bool enabled=true;

	float rigidity=1.0f;

	//Addinational Properties
	float distance=0.0f; //it makes distance joint
	bool grooveEnabled=false; // it makes groove

	QWorld *world;
public:
	/**
	 * Creates a joint between two rigid bodies. Anchor points will be locked to the corresponding position on the QRigidBody if the relevant QRigidBody is defined. If not defined, the anchor point will be assumed as a fixed point in space.
	 * @param bodyA A body in the world. If set to nullptr, anchorWorldPositionA will be a fixed point in space. 
	 * @param anchorWorldPositionA The anchorA position in world coordinates.
	 * @param anchorWorldPositionB The anchorB position in world coordinates. 
	 * @param bodyB Another body in the world. If set to nullptr, anchorWorldPositionB will be a fixed point in space.
	 * @remarks Two anchor points are defined, and the default distance value is the distance between the two points.
	 */
	QJoint(QRigidBody *bodyA,QVector anchorWorldPositionA,QVector anchorWorldPositionB,QRigidBody* bodyB=nullptr); // There are defined two achor points. Default distance value is the distance between the two points.
	/**
	 * Creates a joint between two rigid bodies with defined a single position to anchors. When you create a QJoint with an anchor parameter fixed to a single position, you obtain a QJoint with a constraint distance of 0, commonly known as a pin joint. If one of the QRigidBody parameters is not defined, the constraint will be applied not between two rigid body objects but between one rigid body object and an imaginary fixed point in space. 
	 * @param bodyA A body in the world. If set to nullptr, anchorWorldPositionA will be a fixed point in space. 
	 * @param commonAnchorWorldPosition The common anchor position to set anchors of the joint in world coordinates. 
	 * @param bodyB Another body in the world. If set to nullptr, anchorWorldPositionB will be a fixed point in space.
	 * @remarks Two anchor points are defined, and the default distance value is the distance between the two points.
	 */
	QJoint(QRigidBody *bodyA,QVector commonAnchorWorldPosition,QRigidBody* bodyB=nullptr) : QJoint(bodyA,commonAnchorWorldPosition,commonAnchorWorldPosition,bodyB){}; // There is a defined common anchor position, default distance is the zero.
	~QJoint();

	//Get Methods
	/** Returns bodyA of the joint. */
	inline QRigidBody * GetBodyA(){
		return bodyA;
	}
	/** Returns bodyB of the joint. */
	inline QRigidBody * GetBodyB(){
		return bodyB;
	}
	/** Returns the position of anchorA of the joint.If bodyA is defined, it returns the relative position of the anchor according to bodyA, otherwise, it returns the position of the anchor in world coordinates. */
	inline QVector GetAnchorAPosition(){
		return anchorA;
	}
	/** Returns the position of anchorB of the joint.If bodyB is defined, it returns the relative position of the anchor according to bodyA, otherwise, it returns the position of the anchor in world coordinates. */
	inline QVector GetAnchorBPosition(){
		return anchorB;
	}
	/** Returns the position of anchorA of the joint in world coordinates. */
	inline QVector GetAnchorAGlobalPosition(){
		return anchorGlobalA;
	}
	/** Returns the position of anchorB of the joint in world coordinates. */
	inline QVector GetAnchorBGlobalPosition(){
		return anchorGlobalB;
	}
	/** Returns whether collisions are enabled between bodies of the joint. */
	inline bool GetCollisionEnabled(){
		return collisionsEnabled;
	}
	/** Returns the rigidity of the joint. */
	inline float GetRigidity(){
		return rigidity;
	}
	/** Returns the distance value of the joint. */
	inline float GetDistance(){
		return distance;
	}
	/** Returns whether the groove mode of the joint enabled or not. */
	inline bool GetGrooveEnabled(){
		return grooveEnabled;
	}

	//Set Methods (It returns joint)
	/** Sets bodyA of the joint.
	 * @param body A pointer of the body to set.
	 * @return A pointer to the joint itself.
	 */
	inline QJoint * SetBodyA(QRigidBody *body){
		bodyA=body;
		return this;
	}
	/** Sets bodyB of the joint.
	 * @param body A pointer of the body to set.
	 * @return A pointer to the joint itself.
	 */
	inline QJoint * SetBodyB(QRigidBody *body){
		bodyB=body;
		return this;
	}
	/** Sets anchorA position of the joint.
	 * @param worldPosition A position to set in world coordinates.
	 * @return A pointer to the joint itself.
	 */
	inline QJoint * SetAnchorAPosition(QVector worldPosition){
		if(bodyA!=nullptr){
			this->anchorA=(worldPosition-bodyA->GetPosition()).Rotated(-bodyA->GetRotation());
		}else{
			this->anchorA=worldPosition;
		}
		return this;
	}
	/** Sets anchorB position of the joint.
	 * @param worldPosition A position to set in world coordinates.
	 * @return A pointer to the joint itself.
	 */
	inline QJoint * SetAnchorBPosition(QVector worldPosition){
		if(bodyB!=nullptr){
			this->anchorB=(worldPosition-bodyB->GetPosition()).Rotated(-bodyB->GetRotation());
		}else{
			this->anchorB=worldPosition;
		}
		return this;
	}

	/** Sets the rigidity of the joint.
	 * @param value The rigidity value to set. It must be a value between 0.0 and 1.0.
	 * @return A pointer to the joint itself.
	 */
	inline QJoint* SetRigidity(float value){
		rigidity=value;
		return this;
	}
	/** Sets the distance value of the joint.
	 * @param value The distance value to set.  
	 * @return A pointer to the joint itself.
	 */
	inline QJoint* SetDistance(float value){
		distance=value;
		return this;
	}

	/** Sets whether the groove mode of the joint enabled or not. When groove mode is enabled in QJoints, distance constraint will only be applied when the set distance is exceeded.
	 * @param value True or false.  
	 * @return A pointer to the joint itself.
	 */
	inline QJoint* SetGrooveEnabled(bool value){
		grooveEnabled=value;
		return this;
	}

	/** Sets whether collisions are enabled between bodies of the joint.
	 * @param value True or false. 
	 * @return A pointer to the joint itself.
	 */
	QJoint* SetCollisionEnabled(bool value);



	/** Updates the constraints of the joint. */
	virtual void Update();

	friend class QWorld;




};



#endif // QJOINT_H
