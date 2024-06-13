if (NOT APPLE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=default")
endif()

add_library(sipp
  temporal_graph.cpp
)
target_include_directories(sipp PUBLIC 
  ${Eigen_INCLUDE_DIRS}
)
target_link_libraries(sipp
  trajopt
)

add_subdirectory(test)