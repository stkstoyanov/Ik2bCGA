project(GaaletTests)

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
     ${SRC_ROOT}/main.cpp
     ${SRC_ROOT}/cga.cpp
   )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(${INC_DIR})
add_executable(${PROJECT_NAME} ${SRC_DIR})
