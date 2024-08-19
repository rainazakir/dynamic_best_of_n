if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/argos3/plugins/robots/kilobot/control_interface)

add_executable(agent_stubmain agent.h agent_main.c)
add_executable(agent_stubblue agent.h agent_blue.c)

if(UNIX AND NOT APPLE)
  target_link_libraries(agent_stubmain  argos3plugin_simulator_kilolib m)
  target_link_libraries(agent_stubblue  argos3plugin_simulator_kilolib m)

else(UNIX AND NOT APPLE)
  target_link_libraries(agent_stubmain  argos3plugin_simulator_kilolib)
  target_link_libraries(agent_stubblue  argos3plugin_simulator_kilolib)

endif()

endif(ARGOS_BUILD_FOR_SIMULATOR)
