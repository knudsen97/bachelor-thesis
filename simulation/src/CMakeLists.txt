add_library(src MODULE test_controller.cpp  
camera.cpp 
planner.cpp
bug.cpp
)
include_directories(${simulation_SOURCE_DIR}/inc)

target_link_libraries(src
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_entities
  ${OpenCV_LIBS} 
  )
