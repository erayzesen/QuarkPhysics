#!/bin/bash
echo " Running QuarkPhysics Examples... "
build_flag=""
if [ "$1" == "-r"  -o  "$1" == "-release"  ]
then
	build_flag="-DCMAKE_BUILD_TYPE=Release"
	echo "BUILD MODE = RELEASE"
else 
	build_flag=""
fi
rm -rf build
mkdir build
cd build
cmake $build_flag ..
cmake --build .
./QuarkPhysics
