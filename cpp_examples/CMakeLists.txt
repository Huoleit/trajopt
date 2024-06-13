include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}/include
)

set(PROJECT_DATA_DIR "${PROJECT_SOURCE_DIR}/data")
set(PROJECT_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/config")

configure_file (
  "${CMAKE_CURRENT_SOURCE_DIR}/get_dir.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/get_dir.h" @ONLY
)

add_executable(arm_cpp arm.cpp)
target_link_libraries(arm_cpp trajopt osgviewer)

add_executable(armload armload.cpp)
target_link_libraries(armload trajopt osgviewer)

add_executable(dual_arm dual_arm.cpp)
target_link_libraries(dual_arm trajopt osgviewer)

add_executable(time_optimal_no_collision time_optimal_no_collision.cpp)
target_link_libraries(time_optimal_no_collision trajopt osgviewer)

add_executable(spatial_temporal_opt spatial_temporal_opt.cpp)
target_link_libraries(spatial_temporal_opt sipp osgviewer)

add_executable(with_time_profile with_time_profile.cpp)
target_link_libraries(with_time_profile sipp osgviewer)