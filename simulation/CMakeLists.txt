cmake_minimum_required(VERSION 2.8.12)
project(simulation)

if(APPLE)
  option(ASAN "Use Clang Address Sanitizer in Debug builds")
  if(ASAN)
    # Add address sanitizer support for CLang
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fsanitize=address -fno-optimize-sibling-calls -fno-omit-frame-pointer")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address -fno-optimize-sibling-calls -fno-omit-frame-pointer")
    set(CMAKE_MODULE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} -fsanitize=address")
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} -fsanitize=address")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} -fsanitize=address")
  endif(ASAN)
endif(APPLE)

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(ARGoS REQUIRED)
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})

# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
#include(ARGoSCheckQTOpenGL)

# Look for the (optional) galib library
find_package(GALIB)
if(GALIB_FOUND)
  include_directories(${GALIB_INCLUDE_DIRS})
endif(GALIB_FOUND)

# Find Lua
find_package(Lua REQUIRED)
include_directories(${LUA_INCLUDE_DIR})

# Find opencv
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS}) # Not needed for CMake >= 2.8.11


#add matlab
find_package(Python2 COMPONENTS Development NumPy)

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${LUA_INCLUDE_DIR})

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS})

# Descend into the inc and src directory

add_library(src MODULE 
  ${simulation_SOURCE_DIR}/src/test_controller.cpp
  ${simulation_SOURCE_DIR}/src/camera.cpp 
  ${simulation_SOURCE_DIR}/src/planner.cpp
  ${simulation_SOURCE_DIR}/src/bug.cpp
  ${simulation_SOURCE_DIR}/src/controller.cpp
  ${simulation_SOURCE_DIR}/src/cameraServerLoop.cpp
  ${simulation_SOURCE_DIR}/src/masterLoopFunction.cpp
  ${simulation_SOURCE_DIR}/src/protocol.cpp
  ${simulation_SOURCE_DIR}/src/sort_loop_function.cpp
  ${simulation_SOURCE_DIR}/inc/matplotlibcpp.h
  ${simulation_SOURCE_DIR}/src/swarmManager.cpp
)


include_directories(${simulation_SOURCE_DIR}/inc PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})

target_link_libraries(
  src
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_entities
  ${OpenCV_LIBS} 
  Python2::Python Python2::NumPy
  )



# Descend into the embedding directory
#add_subdirectory(embedding)


