#ifndef QPHYSICSRENDERER_H
#define QPHYSICSRENDERER_H
#include "SFML/Graphics.hpp"
#include "QuarkPhysics/qworld.h"

class QPhysicsRenderer
{
public:
	QPhysicsRenderer();
	static void RenderSpatialCells(QWorld *world,sf::RenderWindow *window);
	static void RenderBoundingBoxes(QWorld *world,sf::RenderWindow *window);
	static void RenderColliders(QWorld *world,sf::RenderWindow *window);
	static void RenderJointsAndSprings(QWorld *world,sf::RenderWindow *window);
	static void RenderRaycasts(QWorld *world,sf::RenderWindow *window);
	static void RenderPhysicsGizmos(QWorld *world,sf::RenderWindow *window);

	//COLORS
	inline static sf::Color COLOR_BG{0x211a21ff};
	inline static sf::Color COLOR_BODY_DYNAMIC{0x8bc849ff};
	inline static sf::Color COLOR_BODY_DYNAMIC_KINEMATIC{0x957587ff};
	inline static sf::Color COLOR_BODY_DYNAMIC_SLEEPING{0x5b7a40ff};
	inline static sf::Color COLOR_BODY_STATIC{0x56b9bdff};
	inline static sf::Color COLOR_CONTACT{0xea513aff};
	inline static sf::Color COLOR_JOINT{0xacb765ff};
	inline static sf::Color COLOR_SPRING{0x584836ff};
	inline static sf::Color COLOR_SPRING_INTERNAL{0x2f2723ff};
	inline static sf::Color COLOR_RAYCAST{0x939b80ff};
	inline static sf::Color COLOR_AREA{0x939b80ff};

};

#endif // QPHYSICSRENDERER_H
