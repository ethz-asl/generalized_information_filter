cmake_minimum_required(VERSION 2.8.0)
project(filter_test)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

cs_add_library(filter_test
  src/library/filter.cpp
  src/library/residual.cpp
  src/library/estimator.cpp
  src/library/measurement_manager.cpp
  src/library/problem_builder.cpp
  src/library/block.cpp)

cs_add_executable(filter_test_node
    src/node/filter_test_node.cpp
)

#add_subdirectory(src/test)

target_link_libraries(filter_test_node ${PROJECT_NAME})

catkin_add_gtest(test_state
  src/test/test_state.cpp)
target_link_libraries(test_state ${PROJECT_NAME})

catkin_add_gtest(test_filter
  src/test/test_filter.cpp)
target_link_libraries(test_filter ${PROJECT_NAME})

cs_install()
cs_export()
