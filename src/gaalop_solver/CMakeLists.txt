project(gaalop_solver)

set(${PROJECT_NAME}_VERSION_MAJOR 0)
set(${PROJECT_NAME}_VERSION_MINOR 0)
set(${PROJECT_NAME}_VERSION_PATCH 1)

set(
     INC_DIR
     ${INC_ROOT}
   )

set(
     SRC_DIR
     gaalop_solver.cpg
   )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O2")
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

include_directories(${INC_DIR})
GPC_CXX_ADD_LIBRARY(${PROJECT_NAME} SHARED ${SRC_DIR})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${LIBRARY_NAME})
