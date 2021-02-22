add_library(test_controller MODULE test_controller.h 
test_controller.cpp 
camera.cpp camera.h 
planner.cpp planner.h
bug.cpp bug.h
)
link_directories(${CMAKE_SOURCE_DIR}/argos3-test/utils)
include_directories(${CMAKE_SOURCE_DIR}/argos3-test/utils)
target_link_libraries(test_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_entities
  ${OpenCV_LIBS} 
  )