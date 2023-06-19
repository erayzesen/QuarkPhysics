
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

#include <SFML/Window/WindowStyle.hpp>
#include <iostream>
#include <chrono>
#include <SFML/Graphics.hpp>
#include "qphysicsrenderer.h"

//Example Scenes
#include "Examples/examplescenebenchmarkboxes.h"
#include "Examples/examplescenebenchmarkboxes2.h"
#include "Examples/examplescenejoints.h"
#include "Examples/examplescenemixedbodies.h"
#include "Examples/examplesceneraycasts.h"
#include "Examples/examplescenesoftbodies.h"
#include "Examples/examplesceneplatformer.h"
#include "Examples/examplescenefrictions.h"
#include "Examples/examplesceneblobs.h""


//Font Resource Binary Header
#include "resources/robotoFont.hpp"

using namespace std;



sf::RenderWindow *window=new sf::RenderWindow();
sf::Font *font;

//Scenes
QExampleScene *scene;

//FPS calculating helpers
int frameRateLimit=-1;
float fps=0.0f;
sf::Clock sfClock;
sf::Clock dtClock;
int frameCount=0;
float timeSinceLastUpdate=0.0f;
//Measuring physics world step variables
float worldStepMsSinceLastUpdate=0.0f;
float worldStepMs=0.0f;

//Rendering
bool showCollisions=true;
bool showColliders=true;
bool showSpatialBoundingBox=false;

QVector windowSize(1024,600);

//Input
QVector mousePosition;

void LoadExampleScene(int sceneIndex){
	if(scene!=nullptr){
		delete scene;
	}
	switch (sceneIndex) {
	case 1:
		scene=new ExampleSceneMixedBodies(windowSize);
		break;
	case 2:
		scene=new ExampleSceneSoftBodies(windowSize);
		break;
	case 3:
		scene=new ExampleSceneJoints(windowSize);
		break;
	case 4:
		scene=new ExampleSceneFrictions(windowSize);
		break;
	case 5:
		scene=new ExampleSceneRaycasts(windowSize);
		break;
	case 6:
		scene=new ExampleScenePlatformer(windowSize);
		break;
	case 7:
		scene=new ExampleSceneBenchmarkBoxes(windowSize);
		break;
	case 8:
		scene=new ExampleSceneBenchmarkBoxes2(windowSize);
		break;
	case 9:
		scene=new ExampleSceneBlobs(windowSize);
		break;
	default:
		break;
	}

	scene->world->Update();


}

void CalculateFPS(){
	sf::Time deltaTime=sfClock.restart();
	timeSinceLastUpdate+=deltaTime.asSeconds();

	frameCount+=1;
	if(timeSinceLastUpdate>1.0f){
		fps=frameCount/timeSinceLastUpdate;
		worldStepMs=worldStepMsSinceLastUpdate/frameCount;
		timeSinceLastUpdate=0.0f;
		worldStepMsSinceLastUpdate=0.0f;
		frameCount=0;
		cout << "ms per world step: "<<worldStepMs <<endl;
	}
}

void RenderFPS(QWorld *world){




	//Draw FPS
	sf::Text text;
	text.setFont(*font);
	int fpsInt=floor(fps);
	string str="Press 1...9 key to navigate between example scenes.";
	str+="\n FPS:"+to_string(fpsInt)+"(World Step "+ to_string(worldStepMs)+" ms)" + " | object count:"+ to_string(world->GetBodyCount());
	str+="\n Broadphase:" + to_string(scene->world->GetEnableBroadphase())+" | Sleeping Mode:"+to_string(scene->world->GetEnableSleeping());
	str+="\n Iteration Count:"+to_string(scene->world->GetIterationCount());

	text.setString( str);
	text.setCharacterSize(12);
	text.setFillColor(QPhysicsRenderer::COLOR_BODY_DYNAMIC_KINEMATIC);

	sf::FloatRect backgroundRect=text.getLocalBounds();
	sf::RectangleShape infoBg( sf::Vector2f(backgroundRect.width,backgroundRect.height));
	infoBg.setFillColor(QPhysicsRenderer::COLOR_BG);


	window->draw(infoBg);
	window->draw(text);
}







void FrameLoop(){

	window->clear(QPhysicsRenderer::COLOR_BG);
	if(scene!=nullptr){


		//Measuring World step time for profiling
		auto t1=chrono::high_resolution_clock::now();
		//PHYSICS WORLD STEP
		scene->world->Update();

		auto t2=chrono::high_resolution_clock::now();
		auto elapsed=chrono::duration_cast<chrono::microseconds>(t2-t1).count();
		worldStepMsSinceLastUpdate+=elapsed;
		//End of Measuring


		//RENDERING
		if (showSpatialBoundingBox){
			QPhysicsRenderer::RenderSpatialCells(scene->world,window);
			QPhysicsRenderer::RenderBoundingBoxes(scene->world,window);
		}
		if(showColliders){
			QPhysicsRenderer::RenderColliders(scene->world,window);
			QPhysicsRenderer::RenderJointsAndSprings(scene->world,window);
			QPhysicsRenderer::RenderRaycasts(scene->world,window);
		}
		if(showCollisions)
			QPhysicsRenderer::RenderPhysicsGizmos(scene->world,window);
		RenderFPS(scene->world);
		//Calling OnUpdate Event of the Scene
		scene->OnUpdate();
	}


	window->display();
}

int KeyCodeToNumber(sf::Keyboard::Key &keyCode){
	int res=-1;
	switch(keyCode){
	case sf::Keyboard::Num0 :
		res=0;
		break;
	case sf::Keyboard::Num1 :
		res=1;
		break;
	case sf::Keyboard::Num2 :
		res=2;
		break;
	case sf::Keyboard::Num3 :
		res=3;
		break;
	case sf::Keyboard::Num4 :
		res=4;
		break;
	case sf::Keyboard::Num5 :
		res=5;
		break;
	case sf::Keyboard::Num6 :
		res=6;
		break;
	case sf::Keyboard::Num7 :
		res=7;
		break;
	case sf::Keyboard::Num8 :
		res=8;
		break;
	case sf::Keyboard::Num9 :
		res=9;
		break;
	default:
		break;

	}
	return res;
}

int main()
{

	font=new sf::Font();
	if(!font->loadFromMemory(&roboto_ttf,roboto_ttf_len)){
		cout<<"font error!"<<std::endl;
	}

	sf::ContextSettings settings;
	//settings.antialiasingLevel=8;

	window->create(sf::VideoMode(windowSize.x, windowSize.y), "QuarkPhysics Examples!",sf::Style::Titlebar | sf::Style::Close,settings);
	window->setFramerateLimit(60);
	window->setKeyRepeatEnabled(false);

	LoadExampleScene(1);

	while (window->isOpen())
	{
		sf::Vector2<int> mouseWindowPosition=sf::Mouse::getPosition(*window);
		sf::Vector2<float> mouseGlobalPosition=window->mapPixelToCoords(mouseWindowPosition);
		QVector mousePosition(mouseGlobalPosition.x,mouseGlobalPosition.y);
		CalculateFPS();

		FrameLoop();

		sf::Event event;
		while (window->pollEvent(event))
		{


			if (event.type == sf::Event::Closed){
				delete scene;
				window->close();
			}
			//Mouse Events
			if(event.type==sf::Event::MouseButtonPressed){
				scene->OnMousePressed(mousePosition);
			}else if(event.type==sf::Event::MouseButtonReleased){
				scene->OnMouseReleased(mousePosition);
			}
			if(event.type==sf::Event::MouseMoved){
				scene->OnMouseMoved(mousePosition);
			}
			//Window Resize Events
//			if(event.type==sf::Event::Resized){
//				sf::View view=window->getDefaultView();
//				view.setSize({
//					static_cast<float>(event.size.width),
//					static_cast<float>(event.size.height)
//				});
//				window->setView(view);
//			}
			//Keyboard Events

			if(event.type==sf::Event::KeyPressed){
				scene->OnKeyPressed(event.key.code);
			}

			if(event.type==sf::Event::KeyReleased) {
				int numFromKeyCode=KeyCodeToNumber(event.key.code);
				if(numFromKeyCode>0){
					LoadExampleScene(numFromKeyCode);
				}
				scene->OnKeyReleased(event.key.code);

//				if(event.key.code==sf::Keyboard::H){
//					showColliders=showColliders==true ? false:true;
//				}else if(event.key.code==sf::Keyboard::Left){
//					int newIt=scene->world->GetIterationCount();
//					newIt-=scene->world->GetIterationCount()>1 ? 1:0;
//					scene->world->SetIterationCount(newIt);
//				}else if(event.key.code==sf::Keyboard::Right){
//					int newIt=scene->world->GetIterationCount();
//					newIt+=1;
//					scene->world->SetIterationCount(newIt);
//				}else if(event.key.code==sf::Keyboard::B){
//					bool enableBroadphase=scene->world->GetEnableBroadphase()==false ? true:false;
//					scene->world->SetEnableBroadphase(enableBroadphase);
//				}else if(event.key.code==sf::Keyboard::S){
//					bool enableSleeping=scene->world->GetEnableSleeping()==false ? true:false;
//					scene->world->SetEnableSleeping(enableSleeping);
//				}else if(event.key.code==sf::Keyboard::C){
//					showCollisions=showCollisions==false ? true:false;
//				}else if(event.key.code==sf::Keyboard::F){
//					showSpatialBoundingBox=showSpatialBoundingBox==true ? false:true;
//				}


			}
		}


	}

	return 0;
}



