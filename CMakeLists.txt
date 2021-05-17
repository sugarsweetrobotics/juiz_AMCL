cmake_minimum_required(VERSION 3.5)
project(amcl LANGUAGES CXX C VERSION 0.0.1)
set(CMAKE_CXX_STANDARD "17")
set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")


set(OPERATIONS)

find_package(juiz REQUIRED)
include_directories(${juiz_INCLUDE_DIR})
link_directories(${juiz_LIB_DIR})

message(${juiz_LIB_DIR})

function(add_operation NAME)

  add_library(${NAME} SHARED ${NAME}.cpp)
  target_link_libraries(${NAME} ${JUIZ_LIBRARIES})
  set_target_properties(${NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
  )
  set_target_properties(${NAME} PROPERTIES PREFIX "")

  set(OPERATIONS ${NAME} ${OPERATIONS})
  set(OPERATIONS ${OPERATIONS} PARENT_SCOPE)
endfunction(add_operation)

set(CONTAINERS)
function(add_container NAME)

  add_library(${NAME} SHARED ${NAME}.cpp)
  target_link_libraries(${NAME} juiz )

  set_target_properties(${NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
  )

  set_target_properties(${NAME} PROPERTIES PREFIX "")
  set(CONTAINERS ${NAME} ${CONTAINERS})
  set(CONTAINERS ${CONTAINERS} PARENT_SCOPE)
endfunction(add_container)

add_container(AMCL)
add_container(AMCL_initialize)

set(PROJECT_MODULES ${OPERATIONS} ${CONTAINERS} ${ECS} PARENT_SCOPE)

INSTALL(TARGETS ${CONTAINERS} ${OPERATIONS} DESTINATION share/juiz/${PROJECT_NAME})
INSTALL(FILES ${PROJECT_NAME}.nkshelf ${PROJECT_NAME}.nkproj DESTINATION share/juiz/${PROJECT_NAME})

