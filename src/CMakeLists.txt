

find_package(juiz REQUIRED)
include_directories(${juiz_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/navigation_amcl/include)
#include_directories(${CMAKE_SOURCE_DIR}/CoppeliaRobotics/b0remoteApiBindings/cpp/msgpack-c/include)
#include_directories(${CMAKE_BINARY_DIR}/CoppeliaRobotics/bluezero/include)
#include_directories(${CMAKE_SOURCE_DIR}/CoppeliaRobotics/bluezero/include)

link_directories(${juiz_LIB_DIR})

set(binding_src ${CMAKE_SOURCE_DIR}/CoppeliaRobotics/b0remoteApiBindings/cpp/b0RemoteApi.cpp)

function(juiz_add_module JUIZ_TARGET)
  message("juiz_add_module( ${JUIZ_TARGET} )")
  set(prefix JUIZ)
  set(flags RELEASE)
  set(singleValues)
  set(multiValues ADDITIONAL_SOURCES LIBRARIES)

  include(CMakeParseArguments)
  cmake_parse_arguments(${prefix}
                   "${flags}"
                   "${singleValues}"
                   "${multiValues}"
                  ${ARGN})
  
  message(" - JUIZ_TARGET: ${JUIZ_TARGET}")
  message(" - JUIZ_ADDITIONAL_SOURCES: ${JUIZ_ADDITIONAL_SOURCES}")
  message(" - JUIZ_LIBRARIES: ${JUIZ_LIBRARIES}")
  message(" - JUIZ_ARGN: ${ARGN}")
  add_library(${JUIZ_TARGET} SHARED  ${JUIZ_TARGET}.cpp ${JUIZ_ADDITIONAL_SOURCES})
  set_target_properties(${JUIZ_TARGET}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
  )
  set_target_properties(${JUIZ_TARGET} PROPERTIES PREFIX "")
  target_link_libraries(${JUIZ_TARGET} ${JUIZ_LIBRARIES})

endfunction(juiz_add_module)


juiz_add_module(AMCL LIBRARIES amcl juiz)
juiz_add_module(AMCL_initialize LIBRARIES amcl juiz ADDITIONAL_SOURCES amcl_wrapper.cpp)
juiz_add_module(AMCL_setMap LIBRARIES amcl juiz ADDITIONAL_SOURCES amcl_wrapper.cpp)
