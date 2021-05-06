set(sort_loop_function_SOURCES
  sort_loop_function.h
  sort_loop_function.cpp)

add_library(sort_loop_function MODULE 
${sort_loop_function_SOURCES})
#target_link_libraries(sort_loop_function
#  argos3core_simulator
#  argos3plugin_simulator_entities
#  argos3plugin_simulator_footbot
#  )
