include_directories(
  SYSTEM
  ${BULLET_INCLUDE_DIRS}  
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${JSON_INCLUDE_DIRS}
)

if (NOT APPLE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=default")
endif()

add_library(trajopt
    traj_plotter.cpp
    trajectory_costs.cpp
    trajectory_linear_constraints.cpp
    kinematic_terms.cpp 
    collision_terms.cpp
    json_marshal.cpp
    problem_description.cpp
    bullet_collision_checker.cpp 
    configuration_space.cpp 
    utils.cpp
    rave_utils.cpp
    collision_checker.cpp
    plot_callback.cpp
    bullet_unity.cpp
)
target_link_libraries(trajopt ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY} sco utils json osgviewer)
target_include_directories(trajopt PUBLIC 
    SYSTEM
    ${BULLET_INCLUDE_DIRS}  
    ${Boost_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${OpenRAVE_INCLUDE_DIRS}
    ${JSON_INCLUDE_DIRS}
)
target_link_directories(trajopt PUBLIC 
    ${OpenRAVE_LIBRARY_DIRS}
)

if (DEBUG_BULLET_GUI)
  set_property(SOURCE bullet_collision_checker.cpp APPEND_STRING PROPERTY COMPILE_FLAGS " -DDEBUG_BULLET_GUI")
endif()

add_subdirectory(test)

include_directories(${PYTHON_NUMPY_INCLUDE_DIR})
boost_python_module(ctrajoptpy trajoptpy.cpp)
target_link_libraries(ctrajoptpy trajopt ${Boost_SYSTEM_LIBRARY} )


