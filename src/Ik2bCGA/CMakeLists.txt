project(Ik2bCGA)

set(${PROJECT_NAME}_VERSION_MAJOR 0)
set(${PROJECT_NAME}_VERSION_MINOR 0)
set(${PROJECT_NAME}_VERSION_PATCH 1)

set(
     INC_DIR
     ${INC_ROOT}
     ${INC_ROOT}/gaalet
   )

set(
     SRC_DIR
     Ik2bCGA.cpp
     ../cga.cpp
   )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

include_directories(${INC_DIR})
add_library(${PROJECT_NAME} SHARED ${SRC_DIR})
