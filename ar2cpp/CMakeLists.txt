cmake_minimum_required(VERSION 2.8.3)
project(ar2cpp)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    roscpp
  LIBRARIES
		ar2cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(ar2cpp
	include/${PROJECT_NAME}/ar2.h
	include/${PROJECT_NAME}/joint.h
	src/ar2.cpp
	src/joint.cpp
)
target_link_libraries(ar2cpp ${catkin_LIBRARIES})