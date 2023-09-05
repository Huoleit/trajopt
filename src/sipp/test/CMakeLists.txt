include_directories(${GTEST_INCLUDE_DIRS}  ${Boost_INCLUDE_DIRS})
add_executable(test_sipp
  test_sipp.cpp
)
target_link_libraries(test_sipp 
  sipp
  gtest_main
)
target_compile_definitions(test_sipp PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data/")
gtest_add_tests(TARGET  test_sipp)

add_executable(plot_collision_geometries
  plot_collision_geometries.cpp
)
target_link_libraries(plot_collision_geometries 
  sipp
)

add_executable(test_safe_interval_construction
  test_safe_interval_construction.cpp
)
target_link_libraries(test_safe_interval_construction 
  sipp
  gtest_main
)
gtest_add_tests(TARGET  test_safe_interval_construction)

