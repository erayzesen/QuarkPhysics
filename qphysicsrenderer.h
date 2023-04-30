
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
