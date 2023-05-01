![Quark Physics](images/logo.png)

QuarkPhysics is a 2D physics engine designed for games. Its goal is to provide a reasonable approach to simulate rigid bodies, soft bodies, and different physics models together.

[Examples](https://github.com/erayzesen/QuarkPhysics/tree/master/examples) | [API Documentation]([http://](https://erayzesen.github.io/QuarkPhysics/html/)) | wiki(coming soon)

**Note:** The project is in the development phase until v1.0.  

 ---
 ![Example 01](images/example_01.gif)
 ![Example 02](images/example_02.gif)
 ![Example 02](images/example_04.gif)
 ![Example 02](images/example_05.gif)
 ![Example 03](images/example_03.gif)
 ![Example 02](images/example_06.gif)



## Features
* General Features
  * Primitive shape types (circle, polygon, rectangle...etc) 
  * Physical properties (mass, area, restitution etc.)
  * The API is designed specifically for 2D video games. 
  * Raycasting
  * Collision layer masks for advanced collision filtering
  * SAP for broadphase
  * Supports sleeping islands to improve CPU performance.
  * Flexible and advanced event system.
  * It uses pixels directly as a unit without any abstractions.
  * Unlimited shape-mesh support for bodies.
  * Simple and consistent API
  * QMesh Editor app for editing mesh features of the bodies (coming soon) 
   
* Rigid bodies
  * Joints to connect bodies
  * Reasonable stability for stacked objects.
  * Kinematic bodies for creating controllable physics objects. 
  * Area bodies for detecting and reporting collisions.

* Soft Bodies
  * Springs to connect particles.
  * Mass-spring model.
  * Area-volume preserving model.
  * Shape matching features.
  * Self particle collisions.
  * PBD dynamics.
  * Internal springs and internal particles for adding more complexity to soft body simulations.
  * Customizable constraints for adding more control to simulations.
  * Advanced particle methods. 
  
## Building Examples
You need to install [SFML](https://www.sfml-dev.org/) and [CMake](https://cmake.org/) on your system before. 

Download project, enter the main folder and call this;

        ./build.sh
Another way is that compiling the project directly via gcc if you're on linux call; 

        ./run_linux_fast.sh -r

## Using
Copy the "QuarkPhysics" named subfolder in the main folder to your project and use it. 

## Third Party 
- [nlohmann's json](https://github.com/nlohmann/json) for the json parsing. (Importing meshes via *.qmesh files)
- [SFML](https://www.sfml-dev.org/) library for window,input,opengl. 

## Roadmap
* 1.0
  * API revisions 
  * Optimizations
  * Bug fixes 
* 1.1
  * Auto converting a concave polygon to convex polygons 
  * Static-dynamic particles 
  * Breakable spring connections.
  * UV features to QMesh
* 1.2
   * Determinism (with fixed point method)
   * Destructable rigid bodies.
   * Continuous collision dedection (CCD)
* 1.3
    * Fluid dynamics





