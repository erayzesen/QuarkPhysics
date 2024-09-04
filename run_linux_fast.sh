#!/bin/bash
echo " Running QuarkPhysics Examples... "
build_flag=""
if [ "$1" = "-r"  -o  "$1" = "-release" ]
then
	build_flag="-O3"
	echo "Selected released version. The optimization flag setted as -O3 "
else
	echo "Selected debug version..."
fi
echo "Compiling..."
rm -rf build
mkdir build
cd build
gcc $build_flag -c ../QuarkPhysics/*.cpp ../QuarkPhysics/extensions/*.cpp  ../QuarkPhysics/json/*.hpp ../examples/*.cpp ../resources/*.hpp  ../*.cpp
g++ -o QuarkPhysics ./*.o -lsfml-graphics -lsfml-window -lsfml-system
rm *.o
echo "Build Succesfully! Running execute file..."
./QuarkPhysics
#nohup ./QuarkPhysics & 
