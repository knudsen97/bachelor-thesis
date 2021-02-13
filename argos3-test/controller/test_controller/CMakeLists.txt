add_library(test_controller MODULE test_controller.h test_controller.cpp utils.cpp utils.h)
link_directories(/home/claus/bachelor-thesis/argos3-test/utils)
include_directories(/home/claus/bachelor-thesis/argos3-test/utils)
target_link_libraries(test_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_entities
  ${OpenCV_LIBS} 
  )
