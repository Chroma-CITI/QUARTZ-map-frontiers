cmake_minimum_required(VERSION 2.8.3)
project(test_scripts)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14 -Wall -Wextra)


############
# BINARIES #
############

# Add the evaluation_maps executable, but do not compile it into the library.
cs_add_executable(evaluation_maps
  src/evaluation_maps.cpp
)
# Link the evaluation_maps executable against the library.
# target_link_libraries(evaluation_maps ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
