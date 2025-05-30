cmake_minimum_required(VERSION 3.5)

project(QuarkPhysics VERSION 1.0)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
file(GLOB SOURCE_FILES 
	${PROJECT_SOURCE_DIR}/QuarkPhysics/*.cpp
	${PROJECT_SOURCE_DIR}/QuarkPhysics/extensions/*.cpp
	${PROJECT_SOURCE_DIR}/QuarkPhysics/json/*.hpp
	${PROJECT_SOURCE_DIR}/QuarkPhysics/polypartition/*.cpp
	${PROJECT_SOURCE_DIR}/examples/*.cpp 
	${PROJECT_SOURCE_DIR}/*.cpp 
	${PROJECT_SOURCE_DIR}/resources/*.hpp 
)
#if(NOT CMAKE_BUILD_TYPE)
#  set(CMAKE_BUILD_TYPE Release)
#endif()
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_executable(${PROJECT_NAME} ${SOURCE_FILES} )

target_link_libraries(${PROJECT_NAME} sfml-graphics sfml-window sfml-system)

