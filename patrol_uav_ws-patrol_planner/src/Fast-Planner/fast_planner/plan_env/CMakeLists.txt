cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
  tf2_ros
  mavros_msgs
  std_srvs
)
find_package(Threads REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs tf2_ros mavros_msgs std_srvs
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( plan_env 
    src/sdf_map.cpp 
    src/obj_predictor.cpp 
    src/edt_environment.cpp 
    src/raycast.cpp
    src/observation_worker.cpp
    src/observation_integration.cpp
    )
target_link_libraries( plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )  
if(catkin_EXPORTED_TARGETS)
  add_dependencies(plan_env ${catkin_EXPORTED_TARGETS})
endif()

add_executable(obj_generator
    src/obj_generator.cpp 
    )
target_link_libraries(obj_generator 
    ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_search_region test/test_search_region.cpp)
  catkin_add_gtest(test_vertical_obstacle_support test/test_vertical_obstacle_support.cpp)
  catkin_add_gtest(test_observation_grid test/test_observation_grid.cpp)
endif()
