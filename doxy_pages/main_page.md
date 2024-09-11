
/*! \mainpage API Documentation

QuarkPhysics is a 2D physics engine designed for games. Its goal is to provide a reasonable approach to simulate rigid bodies, soft bodies, and different physics models together.

[Project Github Page](https://github.com/erayzesen/QuarkPhysics)

## Concept Map
\htmlonly
<div style="text-align: center; background-color: transparent; max-width: 500px">
  <object type="image/svg+xml" data="../doxy_pages/images/concept.svg" style="width: 100%; height: auto; background-color: transparent;">
    Your browser does not support SVG.
  </object>
</div>

\endhtmlonly


  
## Building Examples
You need to install [SFML](https://www.sfml-dev.org/) and [CMake](https://cmake.org/) on your system before. 

Download project, enter the main folder and call this;

        ./build.sh -r
Another way is that compiling the project directly via gcc if you're on linux call; 

        ./run_linux_fast.sh -r

## Using
Copy the "QuarkPhysics" named subfolder in the main folder to your project and use it. 

## Third Party 
- [nlohmann's json](https://github.com/nlohmann/json) for the json parsing. (Importing meshes via *.qmesh files)
- [SFML](https://www.sfml-dev.org/) library for window,input,opengl. 

## License
Licensed under the [MIT](https://github.com/erayzesen/QuarkPhysics/blob/master/LICENSE) license. 
