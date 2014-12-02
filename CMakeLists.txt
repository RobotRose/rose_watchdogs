cmake_minimum_required(VERSION 2.8.3)
project(rose_watchdogs)

find_package(catkin REQUIRED)

catkin_package(
	INCLUDE_DIRS include
 	LIBRARIES rose_watchdogs
 #  	CATKIN_DEPENDS
 #  	DEPENDS
)

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})

add_library(rose_watchdogs SHARED src/watchdog.cpp)

target_link_libraries(rose_watchdogs ${catkin_LIBRARIES})
