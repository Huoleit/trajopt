include_directories(
    ${CMAKE_SOURCE_DIR}/src
)

add_executable(arm_cpp arm.cpp)
target_link_libraries(arm_cpp trajopt osgviewer)