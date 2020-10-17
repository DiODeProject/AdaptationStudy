if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/argos3/plugins/robots/kilobot/control_interface)

add_executable(agentCDCIlocal_ModelA agentCDCIlocal.h agentCDCIlocal_ModelA.c)
if(UNIX AND NOT APPLE)
	target_link_libraries(agentCDCIlocal_ModelA  argos3plugin_simulator_kilolib rt m) 
else(UNIX AND NOT APPLE)
	target_link_libraries(agentCDCIlocal_ModelA  argos3plugin_simulator_kilolib) 
endif()

add_executable(agentCDCIlocal_ModelB agentCDCIlocal.h agentCDCIlocal_ModelB.c)
if(UNIX AND NOT APPLE)
	target_link_libraries(agentCDCIlocal_ModelB  argos3plugin_simulator_kilolib rt m) 
else(UNIX AND NOT APPLE)
	target_link_libraries(agentCDCIlocal_ModelB  argos3plugin_simulator_kilolib) 
endif()

endif(ARGOS_BUILD_FOR_SIMULATOR)
