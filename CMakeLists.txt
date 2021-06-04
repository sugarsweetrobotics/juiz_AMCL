cmake_minimum_required(VERSION 3.5)

project(mcl LANGUAGES CXX C VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CoppeliaRobotics/bluezero/cmake/Modules/")

add_subdirectory(navigation_amcl)
add_subdirectory(src)


#INSTALL(TARGETS ${CONTAINERS} ${OPERATIONS} DESTINATION share/juiz/${PROJECT_NAME})
#INSTALL(FILES ${PROJECT_NAME}.nkshelf ${PROJECT_NAME}.nkproj DESTINATION share/juiz/${PROJECT_NAME})

