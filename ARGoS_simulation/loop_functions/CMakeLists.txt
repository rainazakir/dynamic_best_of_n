include_directories(argos3/plugins/robot/kilobot/control_interface argos3/plugins/robot/kilobot/simulator)

add_library(kilogrid_stub MODULE kilogrid_stub.h kilogrid_stub.cpp)
target_link_libraries(kilogrid_stub
        argos3core_simulator
        argos3plugin_simulator_dynamics2d
        argos3plugin_simulator_entities
        argos3plugin_simulator_media
        argos3plugin_simulator_kilobot
        argos3plugin_simulator_kilolib
        )
