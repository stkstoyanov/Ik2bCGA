cmake_minimum_required(VERSION 2.6)

set(SRC_ROOT ${CMAKE_SOURCE_DIR}/src)
set(INC_ROOT ${CMAKE_SOURCE_DIR}/include)

set(LIBRARY_NAME Ik2bCGA)

set(GPC_JAR "/usr/share/gpc/gaalop/starter-1.0.0.jar")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(GPC)
if (GPC_FOUND)
  add_subdirectory(src/gaalop_solver)
endif (GPC_FOUND)

add_subdirectory(src/gaalet_solver)
add_subdirectory(src/gaalet_tests)
