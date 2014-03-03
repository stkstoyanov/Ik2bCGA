project(gaalop_tests)

set(${PROJECT_NAME}_VERSION_MAJOR 0)
set(${PROJECT_NAME}_VERSION_MINOR 0)
set(${PROJECT_NAME}_VERSION_PATCH 1)

set(
     INC_DIR
     ${INC_ROOT}
   )

set(
     SRC_DIR
     main.cpg
   )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O2")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(${INC_DIR})
GPC_CXX_ADD_EXECUTABLE(${PROJECT_NAME} ${SRC_DIR})
