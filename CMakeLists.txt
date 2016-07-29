cmake_minimum_required(VERSION 2.8.9)
project(projectiveDynamics)

# FIND GLUT

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

# FIND OPENGL

find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

# FIND EIGEN

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_FOUND)
    message(ERROR "EIGEN not found!")
endif(NOT EIGEN3_FOUND)

# FIND GLM

find_package(GLM REQUIRED)
include_directories(${GLM_INCLUDE_DIRS})
link_directories(${GLM_LIBRARY_DIRS})

if(NOT GLM_FOUND)
    message(ERROR "GLM not found!")
endif(NOT GLM_FOUND)

# DISABLE WARNINGS FROM GL3
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

add_executable(projectiveDynamics
    src/main.cpp
    src/shader_utils.h
    src/shader_utils.cpp)

# LINKING

target_link_libraries(projectiveDynamics ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
