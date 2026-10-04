cmake_minimum_required(VERSION 3.10)

# Isaac Sim descriptions of the Unitree G1 robots for mc_isaac (data only)
project(g1_isaac_description VERSION 0.1.0 LANGUAGES NONE)

option(SRC_MODE "Point the descriptions to the USD files of the source tree instead of the installed ones" OFF)

if(SRC_MODE)
  set(G1_USD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/usd")
else()
  set(G1_USD_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/usd")
  install(DIRECTORY usd/ DESTINATION "share/${PROJECT_NAME}/usd")
endif()

# mc_rtc robot module names with a USD (same names as g1_mj_description); g1_23dof and g1_29dof have no USD yet
set(G1_MODELS
    g1_29dof_no_hands
    g1_29dof_revo2
)

# one <module name>.yaml per robot, looked up by mc_isaac in <prefix>/share/mc_isaac
foreach(MODEL IN LISTS G1_MODELS)
  configure_file(mc_isaac/${MODEL}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/mc_isaac/${MODEL}.yaml" @ONLY)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/mc_isaac/${MODEL}.yaml" DESTINATION share/mc_isaac)
endforeach()
