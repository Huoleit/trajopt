add_executable(test_sipp
  sipp.cpp
)
target_link_libraries(test_sipp 
  sipp
)

add_executable(plot_collision_geometries
  plot_collision_geometries.cpp
)
target_link_libraries(plot_collision_geometries 
  sipp
)