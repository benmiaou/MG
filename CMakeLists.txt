project(MG)

cmake_minimum_required(VERSION 2.8)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()
set(CMAKE_C_FLAGS "-std=c++11 -Wall -Wextra ")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenGL REQUIRED)
add_subdirectory(glfw)

include_directories( ${PROJECT_SOURCE_DIR}/glfw/include )
include_directories( ${PROJECT_SOURCE_DIR}/glfw/deps )
include_directories( ${PROJECT_SOURCE_DIR}/eigen3 )
include_directories( ${PROJECT_SOURCE_DIR}/include )
include_directories( ${PROJECT_SOURCE_DIR}/src )

ADD_DEFINITIONS(-std=c++11)
add_definitions(-DPGHP_DIR="${PROJECT_SOURCE_DIR}")

# the list of source files
set(pghp_SRCS
    src/Shader.cpp
    src/WireCube.cpp
    src/Pointcloud.cpp
    src/Mesh.cpp
    src/Meshloader.cpp
    src/Trackball.cpp
    src/Octree.cpp
    src/main.cpp
    src/volonoi.cpp
    glfw/deps/glad.c
)

add_executable(MG ${pghp_SRCS})

target_link_libraries(MG glfw ${GLFW_LIBRARIES} ${OPENGL_LIBRARIES} dl)
