cmake_minimum_required(VERSION 2.8)

set(PROJECT_NAME astar_example)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type.")

# set(CMAKE_VERBOSE_MAKEFILE on)

project(${PROJECT_NAME})

# Set compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)
	message(STATUS "GCC detected, adding compile flags")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -W -pedantic -std=c++11 -lglfw -lGL")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -O3")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
endif(CMAKE_COMPILER_IS_GNUCXX)

find_package(OpenGL REQUIRED)

set(INC_DIR "../include")
include_directories(${INC_DIR})
include_directories(${OPENGL_INCLUDE_DIRS})

IF (WIN32)
set(GLFW3_INCLUDE "" CACHE PATH "GLFW3 include")
set(GLFW3_LIB "" CACHE FILEPATH "GLFW3 lib")
include_directories(${GLFW3_INCLUDE})
ELSE()
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})
ENDIF()


add_executable(${PROJECT_NAME}-0 "${INC_DIR}/astar.hpp" "example-0.cpp")
add_executable(${PROJECT_NAME}-1 "${INC_DIR}/astar.hpp" "example-1.cpp")
add_executable(${PROJECT_NAME}-2 "${INC_DIR}/astar.hpp" "example-2.cpp")
add_executable(${PROJECT_NAME}-3 "${INC_DIR}/astar.hpp" "example-3.cpp")

add_executable(performance "${INC_DIR}/astar.hpp" "performance_test.cpp")

IF (WIN32)
target_link_libraries(${PROJECT_NAME}-3 ${GLFW3_LIB})
ELSE()
target_link_libraries(${PROJECT_NAME}-3 ${GLFW_LIBRARIES})
ENDIF()


target_link_libraries(${PROJECT_NAME}-3 ${OPENGL_LIBRARIES})

install(DIRECTORY ${INC_DIR} DESTINATION .)
