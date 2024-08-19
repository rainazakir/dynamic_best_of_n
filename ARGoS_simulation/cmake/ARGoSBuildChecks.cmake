#
# Find the ARGoS package
#
find_package(PkgConfig)
find_package(ARGoS REQUIRED)

#pkg_check_modules(ARGOS REQUIRED argos3_simulator)
#set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)
#set(CMAKE_INSTALL_PREFIX ${ARGOS_PREFIX} CACHE STRING "Install path prefix, prepended onto install #directories." FORCE)




#
# Check ARGoS
#
find_package(ARGoS REQUIRED)
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})
string(REPLACE "/lib/argos3" "" ARGOS_PREFIX "${ARGOS_LIBRARY_DIR}")
set(CMAKE_INSTALL_PREFIX ${ARGOS_PREFIX} CACHE STRING "Install path prefix, prepended onto install directories." FORCE)


#
# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
#
#include(ARGoSCheckQTOpenGL)

#
# Find Lua
#
#find_package(Lua53 REQUIRED)

#
# Set ARGoS include dir
#
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${GSL_INCLUDE_DIR})
#/Users/a/argos3-kilobot-master/build/plugins/robots/kilobot

#
# Set ARGoS link dir
#
link_directories(${ARGOS_LIBRARY_DIRS} )
